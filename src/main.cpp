#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <cstring>
#include <cstdio>
#include <vector>
#include "esp_coexist.h"
#include "esp_wifi.h"

#include "dashboard_html.h"
#include "CameraDriver.h"
#include "DjiDriver.h"
#include "GoproDriver.h"
#include "InstaDriver.h"
#include "BetaflightBridge.h"

#pragma GCC diagnostic ignored "-Wvolatile"

// ============================================================================
// Configuration Constants
// ============================================================================
static constexpr const char* AP_SSID         = "OpenLogic_CamSwitch";
static constexpr const char* AP_PASS         = "openlogic";
static constexpr const char* BLE_DEVICE_NAME = "OpenLogic_Controller";
static constexpr uint16_t    HTTP_PORT       = 80;
static constexpr uint8_t     DNS_PORT        = 53;

// ============================================================================
// Command Queue for Thread-Safe Execution on loopTask
// ============================================================================
enum CmdType {
    CMD_RECORD_START,
    CMD_RECORD_STOP,
    CMD_RECORD_TOGGLE,
    CMD_KEY_EVENT,
    CMD_SET_MODE,
    CMD_SET_EIS,
    CMD_PUSH_MODE,
    CMD_POWER_SLEEP,
    CMD_WAKE,
    CMD_TOGGLE_SCREEN,
    CMD_GOPRO_HILIGHT,
    CMD_GOPRO_LOCATE_ON,
    CMD_GOPRO_LOCATE_OFF,
    CMD_GOPRO_WIFI_ON,
    CMD_GOPRO_WIFI_OFF,
    CMD_GOPRO_QUERY,
    CMD_SET_RES,
    CMD_SET_FPS,
    CMD_POWER_NORMAL,
    CMD_KEY_REPORT,
    CMD_SEND_MOCK_GPS,
    CMD_INSTA_HEARTBEAT,
    CMD_DJI_PAIR,
    CMD_SELECT_BRAND,
    CMD_CLEAR_MAC,
    CMD_SAVE_DEFAULT,
    CMD_CLEAR_LOGS,
    CMD_BF_BRIDGE_TOGGLE,
    CMD_ENTER_TCP_BRIDGE,
    CMD_EXIT_TCP_BRIDGE,
    CMD_SET_AUTO_ARM,
    CMD_SET_DISARM_DELAY,
    CMD_SET_AUX_CHANNEL,
    CMD_SET_WIFI_OFF_ARM
};

struct CommandItem {
    CmdType type;
    uint8_t value;
    uint16_t extra;
};

static QueueHandle_t g_cmdQueue = nullptr;
static uint32_t s_disarmTargetMs = 0;
static bool s_autoRecStarted = false;

// Persistent Settings & Operating Mode
static bool g_bridgeMode = false;
static bool g_autoRecOnArm = true;
static uint8_t g_disarmDelaySec = 3;
static uint8_t g_auxRecChannel = 0;
static bool g_wifiOffOnArm = false;
static bool g_wifiSuspended = false;
static uint32_t g_wifiRestoreAtMs = 0;

static bool enqueueCommand(CmdType type, uint8_t value = 0, uint16_t extra = 0) {
    if (!g_cmdQueue) return false;
    CommandItem item = { type, value, extra };
    return xQueueSend(g_cmdQueue, &item, 0) == pdTRUE;
}

// ============================================================================
// Global State & Driver Instances
// ============================================================================
static DNSServer dnsServer;
static AsyncWebServer server(HTTP_PORT);
static AsyncWebSocket ws("/ws");
static Preferences prefs;

static DjiDriver g_djiDriver;
static GoproDriver g_goproDriver;
static InstaDriver g_instaDriver;
static BetaflightBridge g_bfBridge;

static CameraDriver* g_drivers[BRAND_COUNT] = {
    &g_djiDriver,
    &g_goproDriver,
    &g_instaDriver
};

static CameraBrand g_activeBrand = BRAND_DJI;
static CameraDriver* g_activeDriver = g_drivers[BRAND_DJI];

// BLE Central Handles (DJI & GoPro)
static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pWriteChar = nullptr;                // DJI 0xFFF5 / GoPro 0x0072 Command write
static NimBLERemoteCharacteristic* pNotifyChar = nullptr;               // DJI 0xFFF4 / GoPro 0x0073 Command notify
static NimBLERemoteCharacteristic* pGoproSettingsWriteChar = nullptr;  // GoPro 0x0074 Settings write
static NimBLERemoteCharacteristic* pGoproSettingsNotifyChar = nullptr; // GoPro 0x0075 Settings notify
static NimBLERemoteCharacteristic* pGoproQueryWriteChar = nullptr;     // GoPro 0x0076 Query write
static NimBLERemoteCharacteristic* pGoproQueryNotifyChar = nullptr;    // GoPro 0x0077 Query notify

// BLE Peripheral Handles (Insta360 GPS Remote & Shared NUS Server)
static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pInstaWriteChar = nullptr;                // 0xCE81 Read/Write
static NimBLECharacteristic* pInstaNotifyChar = nullptr;               // 0xCE82 Read/Notify
static volatile bool g_instaConnected = false;
static uint16_t g_instaConnHandle = 0;
static NimBLEAddress g_stagedInstaAddr;
static uint16_t g_stagedInstaConnHandle = 0;
static volatile bool g_instaConnectPending = false;

static NimBLEAddress targetAddress;
static bool hasTargetAddress = false;
static bool hasSavedMacForActiveBrand = false;
static uint32_t lastScanRestartMs = 0;

// Reboot-to-Switch State
static bool g_pendingRestart = false;
static uint32_t g_restartTargetMs = 0;

// Host-Task Thread-Safety Flags
static volatile bool g_disconnectPending = false;
static volatile uint32_t g_telemetryVersion = 1;

// WKP Wake Broadcast State
static bool isBroadcastingWake = false;
static uint32_t wakeBroadcastStartMs = 0;

// Non-Blocking Connection State Machine
enum ConnStep {
    CONN_IDLE,
    CONN_INITIATE,
    CONN_AWAIT_SETTLE,
    CONN_SECURE,
    CONN_RESOLVE_CHARS,
    CONN_SUBSCRIBE,
    CONN_AWAIT_CCCD,
    CONN_FINALIZE
};
static volatile ConnStep connStep = CONN_IDLE;
static uint32_t connStateTimerMs = 0;

// Thread-Safe Spinlocks
static portMUX_TYPE g_logMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_telemetryMux = portMUX_INITIALIZER_UNLOCKED;

// Ring Buffer for System Logs
struct LogEntry {
    uint32_t id;
    uint32_t ts;
    char tag[16];
    char msg[112];
};
static constexpr size_t LOG_BUFFER_SIZE = 32;
static LogEntry logRing[LOG_BUFFER_SIZE];
static size_t logHead = 0;
static uint32_t nextLogId = 1;
static uint32_t lastWsPushedLogId = 0;

static void addLog(const char* tag, const char* msg) {
    uint32_t now = millis();
    uint32_t id = 0;
    portENTER_CRITICAL(&g_logMux);
    id = nextLogId++;
    LogEntry& e = logRing[logHead % LOG_BUFFER_SIZE];
    e.id = id;
    e.ts = now;
    strncpy(e.tag, tag, sizeof(e.tag) - 1);
    e.tag[sizeof(e.tag) - 1] = '\0';
    strncpy(e.msg, msg, sizeof(e.msg) - 1);
    e.msg[sizeof(e.msg) - 1] = '\0';
    logHead++;
    portEXIT_CRITICAL(&g_logMux);

    // Note: Do not print to Serial here because Serial (USB CDC COM7) is used
    // as a transparent serial passthrough bridge for Betaflight Configurator.
    // Logs are streamed in real time via WebSocket and available on the Web UI.
}

// Forward Declarations
static void startBleScan();
static void selectCameraBrand(CameraBrand brand, bool reboot = true);
static void startWkpWakeBroadcast();
static void stopWkpWakeBroadcast();
static void clearSavedMacActiveBrand();

// ============================================================================
// JSON String Escaping Helper
// ============================================================================
static void escapeJsonString(const char* input, char* output, size_t maxLen) {
    if (!input || !output || maxLen == 0) return;
    size_t o = 0;
    for (size_t i = 0; input[i] != '\0' && o + 2 < maxLen; i++) {
        char c = input[i];
        if (c == '"') {
            output[o++] = '\\';
            output[o++] = '"';
        } else if (c == '\\') {
            output[o++] = '\\';
            output[o++] = '\\';
        } else if (c == '\n') {
            output[o++] = '\\';
            output[o++] = 'n';
        } else if (c == '\r') {
            output[o++] = '\\';
            output[o++] = 'r';
        } else if (c == '\t') {
            output[o++] = '\\';
            output[o++] = 't';
        } else if ((uint8_t)c < 0x20) {
            output[o++] = ' ';
        } else {
            output[o++] = c;
        }
    }
    output[o] = '\0';
}

// ============================================================================
// NVS MAC Persistence & Byte-Order Helpers
// ============================================================================
static const char* getBrandMacKey(CameraBrand brand) {
    switch (brand) {
        case BRAND_DJI: return "mac_dji";
        case BRAND_GOPRO: return "mac_gopro";
        case BRAND_INSTA360: return "mac_insta";
        default: return "mac_unknown";
    }
}

static void saveActiveBrandMac(const NimBLEAddress& addr) {
    const uint8_t* raw = addr.getVal(); // Little-endian LSB first from NimBLE
    uint8_t msbMac[6];
    for (int i = 0; i < 6; i++) {
        msbMac[i] = raw[5 - i]; // Reverse to standard MSB first order
    }

    portENTER_CRITICAL(&g_telemetryMux);
    g_activeDriver->setSavedMac(msbMac);
    g_telemetryVersion++;
    portEXIT_CRITICAL(&g_telemetryMux);

    const char* key = getBrandMacKey(g_activeBrand);
    prefs.putBytes(key, msbMac, 6);
    char typeKey[20];
    snprintf(typeKey, sizeof(typeKey), "%s_t", key);
    prefs.putUChar(typeKey, addr.getType());
    hasSavedMacForActiveBrand = true;

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             msbMac[0], msbMac[1], msbMac[2], msbMac[3], msbMac[4], msbMac[5]);
    addLog("NVS", ("Saved paired MAC for " + String(g_activeDriver->getBrandName()) + ": " + String(macStr)).c_str());
}

static bool loadSavedMacForBrand(CameraBrand brand, NimBLEAddress& outAddr) {
    const char* key = getBrandMacKey(brand);
    uint8_t msbMac[6] = {0};
    size_t len = prefs.getBytes(key, msbMac, 6);
    if (len == 6 && (msbMac[0] || msbMac[1] || msbMac[2] || msbMac[3] || msbMac[4] || msbMac[5])) {
        portENTER_CRITICAL(&g_telemetryMux);
        g_drivers[brand]->setSavedMac(msbMac);
        g_telemetryVersion++;
        portEXIT_CRITICAL(&g_telemetryMux);

        char typeKey[20];
        snprintf(typeKey, sizeof(typeKey), "%s_t", key);
        uint8_t addrType = prefs.getUChar(typeKey, BLE_ADDR_PUBLIC);

        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 msbMac[0], msbMac[1], msbMac[2], msbMac[3], msbMac[4], msbMac[5]);
        outAddr = NimBLEAddress(std::string(macStr), addrType);
        return true;
    }
    return false;
}

static void clearSavedMacActiveBrand() {
    const char* key = getBrandMacKey(g_activeBrand);
    prefs.remove(key);
    char typeKey[20];
    snprintf(typeKey, sizeof(typeKey), "%s_t", key);
    prefs.remove(typeKey);
    hasTargetAddress = false;
    hasSavedMacForActiveBrand = false;
    uint8_t zeroMac[6] = {0};

    portENTER_CRITICAL(&g_telemetryMux);
    g_activeDriver->setSavedMac(zeroMac);
    g_telemetryVersion++;
    portEXIT_CRITICAL(&g_telemetryMux);

    if (g_activeBrand != BRAND_INSTA360) {
        if (pClient && pClient->isConnected()) {
            pClient->disconnect();
        }
        addLog("NVS", ("Cleared saved MAC for " + String(g_activeDriver->getBrandName()) + "; rescanning...").c_str());
        startBleScan();
    } else {
        addLog("NVS", "Cleared saved MAC for Insta360 GPS Remote");
    }
}

// ============================================================================
// Multi-UUID BLE Write Callback
// ============================================================================
static bool bleWriteCallback(uint16_t handleUuid16, const uint8_t* data, size_t len, bool withResponse) {
    if (g_activeBrand == BRAND_INSTA360) {
        if (!pInstaNotifyChar || !g_instaConnected) return false;
        return pInstaNotifyChar->notify(data, len, g_instaConnHandle);
    }

    if (!pClient || !pClient->isConnected()) {
        return false;
    }

    NimBLERemoteCharacteristic* targetChar = pWriteChar;
    if (g_activeBrand == BRAND_GOPRO) {
        if (handleUuid16 == 0x0074) {
            targetChar = pGoproSettingsWriteChar;
        } else if (handleUuid16 == 0x0076) {
            targetChar = pGoproQueryWriteChar;
        } else {
            targetChar = pWriteChar; // 0x0072 or default
        }
    }

    if (!targetChar) return false;

    bool withResp = withResponse;
    if (!withResp && !targetChar->canWriteNoResponse()) {
        withResp = true;
    } else if (withResp && !targetChar->canWrite()) {
        withResp = false;
    }
    return targetChar->writeValue(data, len, withResp);
}

// ============================================================================
// BLE Client Callbacks & Characteristic Resolution (DJI & GoPro)
// ============================================================================
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* client) override {
        addLog("BLE", "GATT physical link connected");
    }

    void onDisconnect(NimBLEClient* client, int reason) override {
        // Defer state mutation and scan restart to loopTask
        g_disconnectPending = true;
    }
};

static void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (!pData || length == 0) return;
    uint16_t handleTag = 0;
    if (pChar) {
        if (g_activeBrand == BRAND_GOPRO) {
            std::string uuidStr = pChar->getUUID().toString();
            if (uuidStr.find("0073") != std::string::npos) handleTag = 0x0073;
            else if (uuidStr.find("0075") != std::string::npos) handleTag = 0x0075;
            else if (uuidStr.find("0077") != std::string::npos) handleTag = 0x0077;
            else handleTag = pChar->getHandle();
        } else {
            handleTag = pChar->getHandle();
        }
    }
    portENTER_CRITICAL(&g_telemetryMux);
    g_activeDriver->onGattNotify(handleTag, pData, length);
    g_telemetryVersion++;
    portEXIT_CRITICAL(&g_telemetryMux);
}

static bool resolveCameraCharacteristics() {
    pWriteChar = nullptr;
    pNotifyChar = nullptr;
    pGoproSettingsWriteChar = nullptr;
    pGoproSettingsNotifyChar = nullptr;
    pGoproQueryWriteChar = nullptr;
    pGoproQueryNotifyChar = nullptr;

    if (g_activeBrand == BRAND_DJI) {
        NimBLEUUID djiSvc(static_cast<uint16_t>(0xFFF0));
        NimBLERemoteService* pSvc = pClient->getService(djiSvc);
        if (pSvc) {
            pNotifyChar = pSvc->getCharacteristic(NimBLEUUID(static_cast<uint16_t>(0xFFF4)));
            pWriteChar  = pSvc->getCharacteristic(NimBLEUUID(static_cast<uint16_t>(0xFFF5)));
        }
        return (pSvc && pWriteChar && pNotifyChar);
    } else if (g_activeBrand == BRAND_GOPRO) {
        NimBLEUUID goproSvc(static_cast<uint16_t>(0xFEA6));
        NimBLERemoteService* pSvc = pClient->getService(goproSvc);
        if (pSvc) {
            // Command pair: 0072 (write), 0073 (notify)
            pWriteChar               = pSvc->getCharacteristic(NimBLEUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b"));
            pNotifyChar              = pSvc->getCharacteristic(NimBLEUUID("b5f90073-aa8d-11e3-9046-0002a5d5c51b"));
            // Settings pair: 0074 (write), 0075 (notify)
            pGoproSettingsWriteChar  = pSvc->getCharacteristic(NimBLEUUID("b5f90074-aa8d-11e3-9046-0002a5d5c51b"));
            pGoproSettingsNotifyChar = pSvc->getCharacteristic(NimBLEUUID("b5f90075-aa8d-11e3-9046-0002a5d5c51b"));
            // Query pair: 0076 (write), 0077 (notify)
            pGoproQueryWriteChar     = pSvc->getCharacteristic(NimBLEUUID("b5f90076-aa8d-11e3-9046-0002a5d5c51b"));
            pGoproQueryNotifyChar    = pSvc->getCharacteristic(NimBLEUUID("b5f90077-aa8d-11e3-9046-0002a5d5c51b"));
        }
        return (pSvc && pWriteChar && pNotifyChar &&
                pGoproSettingsWriteChar && pGoproSettingsNotifyChar &&
                pGoproQueryWriteChar && pGoproQueryNotifyChar);
    }
    return false;
}

static void subscribeCameraNotifications() {
    if (pNotifyChar && pNotifyChar->canNotify()) {
        pNotifyChar->subscribe(true, notifyCallback);
        addLog("BLE", "Subscribed to primary notifications");
    }
    if (g_activeBrand == BRAND_GOPRO) {
        if (pGoproSettingsNotifyChar && pGoproSettingsNotifyChar->canNotify()) {
            pGoproSettingsNotifyChar->subscribe(true, notifyCallback);
            addLog("BLE", "Subscribed to GoPro settings notifications (0075)");
        }
        if (pGoproQueryNotifyChar && pGoproQueryNotifyChar->canNotify()) {
            pGoproQueryNotifyChar->subscribe(true, notifyCallback);
            addLog("BLE", "Subscribed to GoPro query notifications (0077)");
        }
    }
}

// ============================================================================
// BLE Scan Callbacks (Central mode)
// ============================================================================
class ScanCallbacks : public NimBLEScanCallbacks {
    void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override {
        if (connStep != CONN_IDLE || (pClient && pClient->isConnected())) return;

        NimBLEAddress addr = advertisedDevice->getAddress();

        // 1. If saved MAC exists for active brand, prioritize matching that MAC immediately
        if (hasSavedMacForActiveBrand && hasTargetAddress) {
            if (addr == targetAddress || memcmp(addr.getVal(), targetAddress.getVal(), 6) == 0) {
                targetAddress = addr;
                char buf[64];
                snprintf(buf, sizeof(buf), "Matched saved %s [%s]", g_activeDriver->getBrandName(), addr.toString().c_str());
                addLog("SCAN", buf);
                NimBLEDevice::getScan()->stop();
                connStep = CONN_INITIATE;
                portENTER_CRITICAL(&g_telemetryMux);
                g_telemetryVersion++;
                portEXIT_CRITICAL(&g_telemetryMux);
                return;
            }
            return;
        }

        // 2. Unpaired / New Hardware mode: Match by brand advertisement heuristics
        const char* devName = advertisedDevice->getName().c_str();
        const uint8_t* mfgData = (const uint8_t*)advertisedDevice->getManufacturerData().data();
        size_t mfgLen = advertisedDevice->getManufacturerData().length();

        uint8_t svc16[2] = {0, 0};
        if (advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xFFF0))) {
            svc16[0] = 0xF0; svc16[1] = 0xFF;
        } else if (advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xFEA6))) {
            svc16[0] = 0xA6; svc16[1] = 0xFE;
        } else if (advertisedDevice->haveServiceUUID()) {
            NimBLEUUID u = advertisedDevice->getServiceUUID();
            const uint8_t* val = u.getValue();
            if (val) { svc16[0] = val[0]; svc16[1] = val[1]; }
        }

        if (g_activeDriver->isMatchingAdvertisement(devName, mfgData, mfgLen, svc16)) {
            char logMsg[96];
            snprintf(logMsg, sizeof(logMsg), "Discovered %s [%s] RSSI=%d",
                     g_activeDriver->getBrandName(),
                     addr.toString().c_str(),
                     advertisedDevice->getRSSI());
            addLog("SCAN", logMsg);

            NimBLEDevice::getScan()->stop();
            targetAddress = addr;
            hasTargetAddress = true;
            connStep = CONN_INITIATE;
            portENTER_CRITICAL(&g_telemetryMux);
            g_telemetryVersion++;
            portEXIT_CRITICAL(&g_telemetryMux);
        }
    }
};

// ============================================================================
// BLE Server Callbacks (Insta360 Peripheral Server)
// ============================================================================
class UnifiedServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        if (g_activeBrand == BRAND_INSTA360) {
            addLog("BLE", "Insta360 camera connected to GPS Remote");
            g_instaConnected = true;
            g_stagedInstaConnHandle = connInfo.getConnHandle();
            g_stagedInstaAddr = connInfo.getAddress();
            g_instaConnectPending = true;
        }
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        uint16_t handle = connInfo.getConnHandle();
        if (g_activeBrand == BRAND_INSTA360 && handle == g_instaConnHandle) {
            addLog("BLE", "Insta360 camera disconnected");
            g_instaConnected = false;
            g_instaConnHandle = 0;
            g_disconnectPending = true;
        }
    }
};

class InstaCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        NimBLEAttValue val = pCharacteristic->getValue();
        if (val.length() > 0) {
            portENTER_CRITICAL(&g_telemetryMux);
            g_instaDriver.onGattNotify(0xCE81, val.data(), val.length());
            g_telemetryVersion++;
            portEXIT_CRITICAL(&g_telemetryMux);
        }
    }
};

// ============================================================================
// Non-Blocking Connection State Machine (Central Mode)
// ============================================================================
static void runConnectionStateMachine(uint32_t now) {
    switch (connStep) {
        case CONN_INITIATE: {
            addLog("BLE", ("Connecting to " + targetAddress.toString() + "...").c_str());

            if (g_activeBrand == BRAND_GOPRO) {
                NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
                NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
            } else {
                NimBLEDevice::setSecurityAuth(0);
            }

            if (!pClient->connect(targetAddress, true, true, true)) {
                addLog("BLE", "GATT connect initiate failed");
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }

            connStateTimerMs = now;
            connStep = CONN_AWAIT_SETTLE;
            g_telemetryVersion++;
            break;
        }

        case CONN_AWAIT_SETTLE: {
            if (pClient->isConnected()) {
                pClient->setConnectionParams(12, 24, 0, 150);
                if (g_activeBrand == BRAND_GOPRO) {
                    addLog("BLE", "Securing connection for GoPro bond/encryption...");
                    if (!pClient->secureConnection()) {
                        addLog("BLE", "GoPro secureConnection failed");
                        pClient->disconnect();
                        connStep = CONN_IDLE;
                        g_telemetryVersion++;
                        startBleScan();
                        break;
                    }
                    connStateTimerMs = now;
                    connStep = CONN_SECURE;
                    g_telemetryVersion++;
                } else {
                    connStep = CONN_RESOLVE_CHARS;
                    g_telemetryVersion++;
                }
            } else if (now - connStateTimerMs >= 3000) {
                addLog("BLE", "GATT connect timed out (3000ms)");
                pClient->cancelConnect();
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
            }
            break;
        }

        case CONN_SECURE: {
            if (now - connStateTimerMs >= 100) {
                connStep = CONN_RESOLVE_CHARS;
                g_telemetryVersion++;
            }
            break;
        }

        case CONN_RESOLVE_CHARS: {
            if (!pClient->isConnected()) {
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }
            if (!resolveCameraCharacteristics()) {
                addLog("BLE", "Required GATT Characteristics not found");
                pClient->disconnect();
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }
            connStep = CONN_SUBSCRIBE;
            g_telemetryVersion++;
            break;
        }

        case CONN_SUBSCRIBE: {
            if (!pClient->isConnected()) {
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }
            subscribeCameraNotifications();
            connStateTimerMs = now;
            connStep = CONN_AWAIT_CCCD;
            g_telemetryVersion++;
            break;
        }

        case CONN_AWAIT_CCCD: {
            if (now - connStateTimerMs >= 100) {
                if (!pClient->isConnected()) {
                    connStep = CONN_IDLE;
                    g_telemetryVersion++;
                    startBleScan();
                    break;
                }
                connStep = CONN_FINALIZE;
                g_telemetryVersion++;
            }
            break;
        }

        case CONN_FINALIZE: {
            // Never execute BLE writes / handshakes inside spinlocks!
            g_activeDriver->onConnected(pClient->getConnHandle(), bleWriteCallback);
            addLog("BLE", "Camera initialized and authenticated");

            saveActiveBrandMac(targetAddress);
            connStep = CONN_IDLE;
            g_telemetryVersion++;
            break;
        }

        default:
            break;
    }
}

static void startBleScan() {
    if (g_activeBrand == BRAND_INSTA360) return;
    if (isBroadcastingWake || connStep != CONN_IDLE) return;
    if (pClient && pClient->isConnected()) return;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true); // Active scan so cameras advertising name/UUID in Scan Response are discovered
    pScan->setInterval(160);     // 160ms interval
    pScan->setWindow(32);        // 32ms window (20% duty cycle while active)
    pScan->setMaxResults(0);
    pScan->start(1500, false);   // Scan for 1.5 seconds then stop to yield radio to Wi-Fi
    lastScanRestartMs = millis();
}

static void selectCameraBrand(CameraBrand brand, bool reboot) {
    if (brand >= BRAND_COUNT) return;

    prefs.putUChar("brand", (uint8_t)brand);
    char buf[80];
    snprintf(buf, sizeof(buf), "Switched brand to %s (saving to NVS)", g_drivers[brand]->getBrandName());
    addLog("NVS", buf);

    if (reboot) {
        addLog("SYSTEM", "Reboot scheduled in 250ms to switch BLE role cleanly...");
        g_pendingRestart = true;
        g_restartTargetMs = millis() + 250;
    }
}

// ============================================================================
// WKP Wake Broadcast (DJI & Wake-Capable Action Cams)
// ============================================================================
static void startWkpWakeBroadcast() {
    NimBLEDevice::getScan()->stop();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->stop();

    uint8_t msbMac[6] = {0};
    bool macFound = false;

    if (hasTargetAddress) {
        const uint8_t* rawVal = targetAddress.getVal();
        for (int i = 0; i < 6; i++) {
            msbMac[i] = rawVal[5 - i];
        }
        macFound = true;
    }
    if (!macFound) {
        const char* key = getBrandMacKey(g_activeBrand);
        if (prefs.getBytes(key, msbMac, 6) == 6 &&
            (msbMac[0] || msbMac[1] || msbMac[2] || msbMac[3] || msbMac[4] || msbMac[5])) {
            macFound = true;
        }
    }

    if (!macFound) {
        addLog("WAKE", "No saved camera MAC found; pair camera once before WKP wake");
        startBleScan();
        return;
    }

    uint8_t wkpPayload[9];
    wkpPayload[0] = 'W';
    wkpPayload[1] = 'K';
    wkpPayload[2] = 'P';
    for (int i = 0; i < 6; i++) {
        wkpPayload[3 + i] = msbMac[5 - i]; // Target camera MAC LSB first
    }

    NimBLEAdvertisementData advData;
    advData.setManufacturerData(wkpPayload, sizeof(wkpPayload));
    pAdv->setAdvertisementData(advData);

    pAdv->setMinInterval(32);
    pAdv->setMaxInterval(48);

    pAdv->start();
    isBroadcastingWake = true;
    wakeBroadcastStartMs = millis();

    char buf[128];
    snprintf(buf, sizeof(buf), "Broadcasting WKP wake for %02X:%02X:%02X:%02X:%02X:%02X (2.5s)",
             msbMac[0], msbMac[1], msbMac[2], msbMac[3], msbMac[4], msbMac[5]);
    addLog("WAKE", buf);
}

static void stopWkpWakeBroadcast() {
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->stop();
    isBroadcastingWake = false;
    addLog("WAKE", "Wake broadcast completed. Resuming scan...");
    if (g_activeBrand != BRAND_INSTA360) {
        startBleScan();
    }
}

// ============================================================================
// State & Log Serialization for HTTP & WebSocket
// ============================================================================
static size_t buildStateJson(char* buf, size_t maxLen) {
    CameraTelemetry t;
    CameraBrand brand;
    const char* brandName;
    bool connected = false;
    bool connecting = false;

    if (g_bridgeMode) {
        connected = false;
        connecting = false;
    } else if (g_activeBrand == BRAND_INSTA360) {
        connected = g_instaConnected;
        connecting = !g_instaConnected;
    } else {
        connected = (pClient && pClient->isConnected());
        connecting = (connStep != CONN_IDLE);
    }

    portENTER_CRITICAL(&g_telemetryMux);
    t = g_activeDriver->getTelemetry();
    brand = g_activeBrand;
    brandName = g_activeDriver->getBrandName();
    portEXIT_CRITICAL(&g_telemetryMux);

    const char* stateStr = "DISCONNECTED";
    if (g_bridgeMode) {
        stateStr = "TCP_BRIDGE";
    } else if (connected) {
        if (t.state == CAM_RECORDING) stateStr = "RECORDING";
        else if (t.state == CAM_SLEEPING) stateStr = "SLEEPING";
        else if (t.state == CAM_CONNECTING) stateStr = "CONNECTING";
        else stateStr = "PAIRED";
    } else if (connecting) {
        stateStr = "CONNECTING";
    }

    char recTimeStr[16];
    uint32_t rm = t.recordTimeSec / 60;
    uint32_t rs = t.recordTimeSec % 60;
    snprintf(recTimeStr, sizeof(recTimeStr), "%02lu:%02lu", (unsigned long)rm, (unsigned long)rs);

    char remTimeStr[16];
    if (t.remainVideoSec > 0) {
        uint32_t rh = t.remainVideoSec / 3600;
        uint32_t rmin = (t.remainVideoSec % 3600) / 60;
        snprintf(remTimeStr, sizeof(remTimeStr), "%02lu:%02lu", (unsigned long)rh, (unsigned long)rmin);
    } else {
        strcpy(remTimeStr, "--:--");
    }

    char escMode[48]; escapeJsonString(t.currentMode, escMode, sizeof(escMode));
    char escFps[48]; escapeJsonString(t.resolutionFps, escFps, sizeof(escFps));
    char escEis[48]; escapeJsonString(t.stabilization, escEis, sizeof(escEis));
    char escTemp[48]; escapeJsonString(t.tempStatus, escTemp, sizeof(escTemp));
    char escHex[140]; escapeJsonString(t.lastHexPacket, escHex, sizeof(escHex));

    uint16_t auxVal = (g_auxRecChannel >= 1 && g_auxRecChannel <= 8) ? g_bfBridge.getAuxChannel(g_auxRecChannel) : 0;

    return snprintf(buf, maxLen,
        "{\"type\":\"state\",\"activeBrand\":%d,\"brandName\":\"%s\",\"stateStr\":\"%s\","
        "\"isRecording\":%s,\"battery\":%u,\"currentMode\":\"%s\","
        "\"resolutionFps\":\"%s\",\"stabilization\":\"%s\",\"tempStatus\":\"%s\","
        "\"recTime\":\"%s\",\"remTime\":\"%s\",\"remMB\":%lu,\"remPhotos\":%lu,"
        "\"mac\":\"%s\",\"packetCount\":%lu,\"lastHex\":\"%s\","
        "\"bridgeMode\":%s,\"bfConnected\":%s,\"bfRxBytes\":%lu,\"bfTxBytes\":%lu,"
        "\"fcConnected\":%s,\"fcArmed\":%s,\"auxValue\":%u,\"autoRecArm\":%s,"
        "\"disarmDelaySec\":%u,\"auxRecCh\":%u,\"wifiOffArm\":%s}",
        (int)brand, brandName, stateStr,
        t.isRecording ? "true" : "false",
        t.batteryPercent, escMode, escFps, escEis, escTemp,
        recTimeStr, remTimeStr,
        (unsigned long)t.remainCapacityMB, (unsigned long)t.remainPhotos,
        t.cameraMac, (unsigned long)t.packetCount, escHex,
        g_bridgeMode ? "true" : "false",
        g_bfBridge.isTcpConnected() ? "true" : "false",
        (unsigned long)g_bfBridge.getRxBytes(),
        (unsigned long)g_bfBridge.getTxBytes(),
        g_bfBridge.isFcConnected() ? "true" : "false",
        g_bfBridge.isArmed() ? "true" : "false",
        (unsigned)auxVal,
        g_autoRecOnArm ? "true" : "false",
        (unsigned)g_disarmDelaySec,
        (unsigned)g_auxRecChannel,
        g_wifiOffOnArm ? "true" : "false"
    );
}

static void sendStateJsonToClient(AsyncWebSocketClient* client) {
    if (!client || !client->canSend()) return;
    char json[1024];
    buildStateJson(json, sizeof(json));
    client->text(json);
}

static void broadcastStateJson() {
    if (ws.count() == 0) return;
    char json[1024];
    buildStateJson(json, sizeof(json));
    ws.textAll(json);
}

struct StateSnapshot {
    uint32_t version;
    CameraState state;
    bool isConnected;
    CameraBrand brand;
    bool bridgeMode;
    bool bfConnected;
    bool fcConnected;
    bool fcArmed;
    uint16_t auxValue;
    uint32_t bfTxBytes;
    uint32_t bfRxBytes;
};
static StateSnapshot s_lastSnapshot{};

static bool hasStateChanged() {
    bool conn = false;
    if (g_bridgeMode) {
        conn = false;
    } else if (g_activeBrand == BRAND_INSTA360) {
        conn = g_instaConnected;
    } else {
        conn = (pClient && pClient->isConnected());
    }

    CameraTelemetry t;
    portENTER_CRITICAL(&g_telemetryMux);
    t = g_activeDriver->getTelemetry();
    portEXIT_CRITICAL(&g_telemetryMux);

    bool bfConn = g_bfBridge.isTcpConnected();
    bool fcConn = g_bfBridge.isFcConnected();
    bool fcArmed = g_bfBridge.isArmed();
    uint16_t auxVal = (g_auxRecChannel >= 1 && g_auxRecChannel <= 8) ? g_bfBridge.getAuxChannel(g_auxRecChannel) : 0;
    uint32_t bfTx = g_bfBridge.getTxBytes();
    uint32_t bfRx = g_bfBridge.getRxBytes();

    uint32_t now = millis();
    static uint32_t s_lastBytesPushMs = 0;
    bool bytesChanged = ((s_lastSnapshot.bfTxBytes != bfTx || s_lastSnapshot.bfRxBytes != bfRx) && (now - s_lastBytesPushMs >= 1000));
    bool auxChanged = (abs((int)s_lastSnapshot.auxValue - (int)auxVal) > 25);

    if (s_lastSnapshot.version != g_telemetryVersion ||
        s_lastSnapshot.state != t.state ||
        s_lastSnapshot.isConnected != conn ||
        s_lastSnapshot.brand != g_activeBrand ||
        s_lastSnapshot.bridgeMode != g_bridgeMode ||
        s_lastSnapshot.bfConnected != bfConn ||
        s_lastSnapshot.fcConnected != fcConn ||
        s_lastSnapshot.fcArmed != fcArmed ||
        auxChanged ||
        bytesChanged) {

        s_lastSnapshot.version = g_telemetryVersion;
        s_lastSnapshot.state = t.state;
        s_lastSnapshot.isConnected = conn;
        s_lastSnapshot.brand = g_activeBrand;
        s_lastSnapshot.bridgeMode = g_bridgeMode;
        s_lastSnapshot.bfConnected = bfConn;
        s_lastSnapshot.fcConnected = fcConn;
        s_lastSnapshot.fcArmed = fcArmed;
        s_lastSnapshot.auxValue = auxVal;
        s_lastSnapshot.bfTxBytes = bfTx;
        s_lastSnapshot.bfRxBytes = bfRx;
        if (bytesChanged) {
            s_lastBytesPushMs = now;
        }
        return true;
    }
    return false;
}

static void flushLogsToWebSocket() {
    if (ws.count() == 0) return;

    uint32_t currentHead;
    portENTER_CRITICAL(&g_logMux);
    currentHead = logHead;
    portEXIT_CRITICAL(&g_logMux);

    if (currentHead == lastWsPushedLogId) return;

    if (lastWsPushedLogId < (currentHead > LOG_BUFFER_SIZE ? currentHead - LOG_BUFFER_SIZE : 0)) {
        lastWsPushedLogId = (currentHead > LOG_BUFFER_SIZE) ? currentHead - LOG_BUFFER_SIZE : 0;
    }

    while (lastWsPushedLogId < currentHead) {
        LogEntry e;
        portENTER_CRITICAL(&g_logMux);
        e = logRing[lastWsPushedLogId % LOG_BUFFER_SIZE];
        portEXIT_CRITICAL(&g_logMux);

        char escTag[32]; escapeJsonString(e.tag, escTag, sizeof(escTag));
        char escMsg[240]; escapeJsonString(e.msg, escMsg, sizeof(escMsg));

        char json[320];
        snprintf(json, sizeof(json),
            "{\"type\":\"log\",\"id\":%lu,\"ts\":%lu,\"tag\":\"%s\",\"msg\":\"%s\"}",
            (unsigned long)e.id, (unsigned long)e.ts, escTag, escMsg);

        ws.textAll(json);
        lastWsPushedLogId++;
    }
}

// ============================================================================
// Command Processor (Drains Queue on loopTask)
// ============================================================================
static void processCommand(const CommandItem& item) {
    switch (item.type) {
        case CMD_RECORD_START:
            g_activeDriver->startRecording();
            g_telemetryVersion++;
            addLog("CMD", "Record Start triggered");
            break;
        case CMD_RECORD_STOP:
            g_activeDriver->stopRecording();
            g_telemetryVersion++;
            addLog("CMD", "Record Stop triggered");
            break;
        case CMD_RECORD_TOGGLE:
            g_activeDriver->toggleShutter();
            g_telemetryVersion++;
            addLog("CMD", "Shutter Toggle triggered");
            break;
        case CMD_KEY_EVENT:
            g_activeDriver->sendKeyEvent();
            g_telemetryVersion++;
            addLog("CMD", "Key / Photo Event triggered");
            break;
        case CMD_SET_MODE:
            g_activeDriver->setMode(item.value);
            g_telemetryVersion++;
            addLog("CMD", "Mode Change triggered");
            break;
        case CMD_SET_EIS:
            g_activeDriver->setStabilization(item.value);
            g_telemetryVersion++;
            addLog("CMD", "EIS Stabilization changed");
            break;
        case CMD_PUSH_MODE:
            g_activeDriver->setPushMode(item.value);
            g_telemetryVersion++;
            addLog("CMD", "Push Mode changed");
            break;
        case CMD_POWER_SLEEP:
            g_activeDriver->powerSleep();
            g_telemetryVersion++;
            addLog("CMD", "Power Sleep command sent");
            break;
        case CMD_WAKE:
            if (g_activeBrand == BRAND_DJI) {
                startWkpWakeBroadcast();
            } else {
                g_activeDriver->wakeCamera();
            }
            g_telemetryVersion++;
            break;
        case CMD_TOGGLE_SCREEN:
            g_activeDriver->toggleScreen();
            g_telemetryVersion++;
            addLog("CMD", "Toggle Screen triggered");
            break;
        case CMD_GOPRO_HILIGHT:
            g_activeDriver->tagHilight();
            g_telemetryVersion++;
            addLog("CMD", "GoPro Hilight Tagged");
            break;
        case CMD_GOPRO_LOCATE_ON:
            g_activeDriver->locateCamera(true);
            g_telemetryVersion++;
            addLog("CMD", "GoPro Locate / Beep ON");
            break;
        case CMD_GOPRO_LOCATE_OFF:
            g_activeDriver->locateCamera(false);
            g_telemetryVersion++;
            addLog("CMD", "GoPro Locate / Beep OFF");
            break;
        case CMD_GOPRO_WIFI_ON:
            g_activeDriver->setWifiAp(true);
            g_telemetryVersion++;
            addLog("CMD", "GoPro Wi-Fi AP ON");
            break;
        case CMD_GOPRO_WIFI_OFF:
            g_activeDriver->setWifiAp(false);
            g_telemetryVersion++;
            addLog("CMD", "GoPro Wi-Fi AP OFF");
            break;
        case CMD_SET_RES:
            g_activeDriver->setResolution(item.value);
            g_telemetryVersion++;
            addLog("CMD", "Set Resolution command sent");
            break;
        case CMD_SET_FPS:
            g_activeDriver->setFps(item.value);
            g_telemetryVersion++;
            addLog("CMD", "Set FPS command sent");
            break;
        case CMD_POWER_NORMAL:
            g_activeDriver->powerNormal();
            g_telemetryVersion++;
            addLog("CMD", "Power Normal command sent");
            break;
        case CMD_KEY_REPORT:
            g_activeDriver->sendKeyReport(item.value, item.extra);
            g_telemetryVersion++;
            addLog("CMD", "Key Report sent");
            break;
        case CMD_SEND_MOCK_GPS:
            g_activeDriver->sendMockGps();
            g_telemetryVersion++;
            addLog("CMD", "DJI Mock GPS Data (00/17) sent");
            break;
        case CMD_GOPRO_QUERY:
            g_activeDriver->queryStatus();
            g_telemetryVersion++;
            addLog("CMD", "GoPro Query All Status triggered");
            break;
        case CMD_INSTA_HEARTBEAT:
            g_activeDriver->sendHeartbeat();
            g_telemetryVersion++;
            addLog("CMD", "Insta360 Heartbeat sent");
            break;
        case CMD_DJI_PAIR:
            if (g_activeBrand == BRAND_DJI) {
                reinterpret_cast<DjiDriver*>(g_activeDriver)->sendPairingHandshake(item.value ? item.value : 1);
                g_telemetryVersion++;
                addLog("CMD", "DJI Force Pair Prompt sent");
            }
            break;
        case CMD_SELECT_BRAND:
            selectCameraBrand((CameraBrand)item.value, true);
            break;
        case CMD_CLEAR_MAC:
            clearSavedMacActiveBrand();
            break;
        case CMD_SAVE_DEFAULT:
            prefs.putUChar("brand", (uint8_t)g_activeBrand);
            addLog("NVS", "Saved boot default camera brand to NVS");
            break;
        case CMD_CLEAR_LOGS:
            portENTER_CRITICAL(&g_logMux);
            logHead = 0;
            nextLogId = 1;
            lastWsPushedLogId = 0;
            portEXIT_CRITICAL(&g_logMux);
            if (ws.count() > 0) {
                ws.textAll("{\"type\":\"logs_cleared\"}");
            }
            break;
        case CMD_ENTER_TCP_BRIDGE:
            prefs.putBool("bf_mode", true);
            addLog("SYSTEM", "Entering dedicated TCP Bridge mode (rebooting)...");
            g_pendingRestart = true;
            g_restartTargetMs = millis() + 250;
            break;
        case CMD_EXIT_TCP_BRIDGE:
            prefs.putBool("bf_mode", false);
            addLog("SYSTEM", "Exiting TCP Bridge mode (rebooting)...");
            g_pendingRestart = true;
            g_restartTargetMs = millis() + 250;
            break;
        case CMD_SET_AUTO_ARM:
            g_autoRecOnArm = (item.value != 0);
            prefs.putBool("auto_arm", g_autoRecOnArm);
            addLog("CONFIG", g_autoRecOnArm ? "Auto-Rec on Arm: ENABLED" : "Auto-Rec on Arm: DISABLED");
            g_telemetryVersion++;
            break;
        case CMD_SET_DISARM_DELAY:
            g_disarmDelaySec = (item.value <= 15) ? item.value : 3;
            prefs.putUChar("disarm_dly", g_disarmDelaySec);
            {
                char buf[48];
                snprintf(buf, sizeof(buf), "Disarm Delay set to %u s", (unsigned)g_disarmDelaySec);
                addLog("CONFIG", buf);
            }
            g_telemetryVersion++;
            break;
        case CMD_SET_AUX_CHANNEL:
            g_auxRecChannel = (item.value <= 8) ? item.value : 0;
            prefs.putUChar("aux_ch", g_auxRecChannel);
            {
                char buf[48];
                if (g_auxRecChannel == 0) strcpy(buf, "AUX Rec Switch: OFF");
                else snprintf(buf, sizeof(buf), "AUX Rec Switch: AUX%u", (unsigned)g_auxRecChannel);
                addLog("CONFIG", buf);
            }
            g_telemetryVersion++;
            break;
        case CMD_SET_WIFI_OFF_ARM:
            g_wifiOffOnArm = (item.value != 0);
            prefs.putBool("wifi_arm", g_wifiOffOnArm);
            addLog("CONFIG", g_wifiOffOnArm ? "Wi-Fi Off When Armed: ENABLED" : "Wi-Fi Off When Armed: DISABLED");
            g_telemetryVersion++;
            break;
        case CMD_BF_BRIDGE_TOGGLE:
            if (g_bridgeMode) {
                enqueueCommand(CMD_EXIT_TCP_BRIDGE);
            } else {
                enqueueCommand(CMD_ENTER_TCP_BRIDGE);
            }
            break;
    }
}

static void handleCommandString(const char* cmd, const char* val) {
    if (!cmd || strlen(cmd) == 0) return;

    if (strcmp(cmd, "record_start") == 0) enqueueCommand(CMD_RECORD_START);
    else if (strcmp(cmd, "record_stop") == 0) enqueueCommand(CMD_RECORD_STOP);
    else if (strcmp(cmd, "record_toggle") == 0) enqueueCommand(CMD_RECORD_TOGGLE);
    else if (strcmp(cmd, "key_event") == 0) enqueueCommand(CMD_KEY_EVENT);
    else if (strcmp(cmd, "set_mode") == 0) enqueueCommand(CMD_SET_MODE, val ? atoi(val) : 0);
    else if (strcmp(cmd, "set_eis") == 0) enqueueCommand(CMD_SET_EIS, val ? atoi(val) : 0);
    else if (strcmp(cmd, "push_mode") == 0) enqueueCommand(CMD_PUSH_MODE, val ? atoi(val) : 0);
    else if (strcmp(cmd, "power_sleep") == 0) enqueueCommand(CMD_POWER_SLEEP);
    else if (strcmp(cmd, "wake") == 0) enqueueCommand(CMD_WAKE);
    else if (strcmp(cmd, "toggle_screen") == 0) enqueueCommand(CMD_TOGGLE_SCREEN);
    else if (strcmp(cmd, "gopro_hilight") == 0) enqueueCommand(CMD_GOPRO_HILIGHT);
    else if (strcmp(cmd, "gopro_locate_on") == 0) enqueueCommand(CMD_GOPRO_LOCATE_ON);
    else if (strcmp(cmd, "gopro_locate_off") == 0) enqueueCommand(CMD_GOPRO_LOCATE_OFF);
    else if (strcmp(cmd, "gopro_wifi_on") == 0) enqueueCommand(CMD_GOPRO_WIFI_ON);
    else if (strcmp(cmd, "gopro_wifi_off") == 0) enqueueCommand(CMD_GOPRO_WIFI_OFF);
    else if (strcmp(cmd, "set_res") == 0) enqueueCommand(CMD_SET_RES, val ? atoi(val) : 0);
    else if (strcmp(cmd, "set_fps") == 0) enqueueCommand(CMD_SET_FPS, val ? atoi(val) : 0);
    else if (strcmp(cmd, "power_normal") == 0) enqueueCommand(CMD_POWER_NORMAL);
    else if (strcmp(cmd, "dji_qs_short") == 0) enqueueCommand(CMD_KEY_REPORT, 0x02, 0x0000);
    else if (strcmp(cmd, "dji_qs_long") == 0) enqueueCommand(CMD_KEY_REPORT, 0x02, 0x0001);
    else if (strcmp(cmd, "dji_snapshot") == 0) enqueueCommand(CMD_KEY_REPORT, 0x03, 0x0000);
    else if (strcmp(cmd, "key_report") == 0) enqueueCommand(CMD_KEY_REPORT, val ? atoi(val) : 1, 0x0000);
    else if (strcmp(cmd, "mock_gps") == 0) enqueueCommand(CMD_SEND_MOCK_GPS);
    else if (strcmp(cmd, "gopro_query") == 0) enqueueCommand(CMD_GOPRO_QUERY);
    else if (strcmp(cmd, "dji_pair") == 0) enqueueCommand(CMD_DJI_PAIR, 1);
    else if (strcmp(cmd, "clear_mac") == 0) enqueueCommand(CMD_CLEAR_MAC);
    else if (strcmp(cmd, "save_default") == 0) enqueueCommand(CMD_SAVE_DEFAULT);
    else if (strcmp(cmd, "clear_logs") == 0) enqueueCommand(CMD_CLEAR_LOGS);
    else if (strcmp(cmd, "enter_bridge") == 0) enqueueCommand(CMD_ENTER_TCP_BRIDGE);
    else if (strcmp(cmd, "exit_bridge") == 0) enqueueCommand(CMD_EXIT_TCP_BRIDGE);
    else if (strcmp(cmd, "set_auto_arm") == 0) enqueueCommand(CMD_SET_AUTO_ARM, val ? atoi(val) : 1);
    else if (strcmp(cmd, "set_disarm_delay") == 0) enqueueCommand(CMD_SET_DISARM_DELAY, val ? atoi(val) : 3);
    else if (strcmp(cmd, "set_aux_ch") == 0) enqueueCommand(CMD_SET_AUX_CHANNEL, val ? atoi(val) : 0);
    else if (strcmp(cmd, "set_wifi_off_arm") == 0) enqueueCommand(CMD_SET_WIFI_OFF_ARM, val ? atoi(val) : 0);
    else if (strcmp(cmd, "bf_bridge_toggle") == 0) {
        if (g_bridgeMode) enqueueCommand(CMD_EXIT_TCP_BRIDGE);
        else enqueueCommand(CMD_ENTER_TCP_BRIDGE);
    }
    else if (strcmp(cmd, "select_cam") == 0 || strcmp(cmd, "select_brand") == 0) {
        if (val) enqueueCommand(CMD_SELECT_BRAND, atoi(val));
    }
}

// ============================================================================
// WebSocket Event Handler (async_tcp context -> Enqueue Only)
// ============================================================================
static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        addLog("WS", ("Client #" + String(client->id()) + " connected").c_str());
        sendStateJsonToClient(client);
    } else if (type == WS_EVT_DISCONNECT) {
        addLog("WS", ("Client #" + String(client->id()) + " disconnected").c_str());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            char buf[128] = {0};
            size_t copyLen = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
            memcpy(buf, data, copyLen);
            buf[copyLen] = '\0';

            if (buf[0] == '{') {
                char cmd[32] = {0};
                char val[32] = {0};
                const char* pCmd = strstr(buf, "\"cmd\":");
                if (pCmd) {
                    pCmd += 6;
                    while (*pCmd == ' ' || *pCmd == '"') pCmd++;
                    size_t ci = 0;
                    while (*pCmd && *pCmd != '"' && *pCmd != ',' && *pCmd != '}' && ci < sizeof(cmd) - 1) {
                        cmd[ci++] = *pCmd++;
                    }
                }
                const char* pVal = strstr(buf, "\"val\":");
                if (pVal) {
                    pVal += 6;
                    while (*pVal == ' ' || *pVal == '"') pVal++;
                    size_t vi = 0;
                    while (*pVal && *pVal != '"' && *pVal != ',' && *pVal != '}' && vi < sizeof(val) - 1) {
                        val[vi++] = *pVal++;
                    }
                }
                handleCommandString(cmd, val[0] ? val : nullptr);
            } else {
                handleCommandString(buf, nullptr);
            }
        }
    }
}

// ============================================================================
// Web Server & Captive Portal Setup
// ============================================================================
static void setupWebServer() {
    auto send204 = [](AsyncWebServerRequest *req) { req->send(204); };
    server.on("/favicon.ico", HTTP_ANY, send204);
    server.on("/apple-touch-icon.png", HTTP_ANY, send204);
    server.on("/apple-touch-icon-precomposed.png", HTTP_ANY, send204);
    server.on("/apple-touch-icon-120x120.png", HTTP_ANY, send204);
    server.on("/apple-touch-icon-120x120-precomposed.png", HTTP_ANY, send204);
    server.on("/manifest.json", HTTP_ANY, [](AsyncWebServerRequest *req) { req->send(404); });

    // Captive Portal Probe Interceptions
    auto redirectPortal = [](AsyncWebServerRequest *req) {
        req->redirect("http://192.168.4.1/");
    };
    server.on("/hotspot-detect.html", HTTP_ANY, redirectPortal);
    server.on("/canonical.html", HTTP_ANY, redirectPortal);
    server.on("/generate_204", HTTP_ANY, redirectPortal);
    server.on("/gen_204", HTTP_ANY, redirectPortal);
    server.on("/connecttest.txt", HTTP_ANY, redirectPortal);
    server.on("/ncsi.txt", HTTP_ANY, redirectPortal);
    server.on("/success.txt", HTTP_ANY, redirectPortal);
    server.on("/mobile/status.php", HTTP_ANY, redirectPortal);
    server.on("/library/test/success.html", HTTP_ANY, redirectPortal);

    // Root UI
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse *response = request->beginResponse(
            200, "text/html", (const uint8_t*)DASHBOARD_HTML, sizeof(DASHBOARD_HTML) - 1
        );
        response->addHeader("Connection", "close");
        response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        request->send(response);
    });

    // Fallback API: Live State JSON
    server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* request) {
        char json[1024];
        buildStateJson(json, sizeof(json));
        request->send(200, "application/json", json);
    });

    // Fallback API: Action Dispatcher (Enqueues for execution on loopTask)
    server.on("/api/action", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (!request->hasParam("cmd")) {
            request->send(400, "text/plain", "Missing cmd param");
            return;
        }
        String cmd = request->getParam("cmd")->value();
        String val = request->hasParam("val") ? request->getParam("val")->value() : "";

        handleCommandString(cmd.c_str(), val.length() > 0 ? val.c_str() : nullptr);
        request->send(200, "text/plain", "OK");
    });

    // Fallback API: Camera Brand Selection
    server.on("/api/select_cam", HTTP_ANY, [](AsyncWebServerRequest* request) {
        int brand = -1;
        if (request->hasParam("brand")) {
            brand = request->getParam("brand")->value().toInt();
        } else if (request->hasParam("brand", true)) {
            brand = request->getParam("brand", true)->value().toInt();
        }

        if (brand >= 0 && brand < BRAND_COUNT) {
            enqueueCommand(CMD_SELECT_BRAND, (uint8_t)brand);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing or invalid brand");
        }
    });

    // API: Clear Saved MAC / Rescan
    server.on("/api/clear_mac", HTTP_POST, [](AsyncWebServerRequest* request) {
        enqueueCommand(CMD_CLEAR_MAC);
        request->send(200, "text/plain", "OK");
    });

    // API: Save Boot Default
    server.on("/api/save_default", HTTP_POST, [](AsyncWebServerRequest* request) {
        enqueueCommand(CMD_SAVE_DEFAULT);
        request->send(200, "text/plain", "SAVED");
    });

    // Fallback API: Logs Ring Buffer (Stack copy inside spinlock, format outside)
    server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* request) {
        uint32_t sinceId = 0;
        if (request->hasParam("since")) {
            sinceId = request->getParam("since")->value().toInt();
        }

        LogEntry tempEntries[LOG_BUFFER_SIZE];
        size_t tempCount = 0;

        portENTER_CRITICAL(&g_logMux);
        size_t count = (logHead < LOG_BUFFER_SIZE) ? logHead : LOG_BUFFER_SIZE;
        size_t start = (logHead < LOG_BUFFER_SIZE) ? 0 : (logHead % LOG_BUFFER_SIZE);

        for (size_t i = 0; i < count; i++) {
            size_t idx = (start + i) % LOG_BUFFER_SIZE;
            const LogEntry& e = logRing[idx];
            if (e.id > sinceId) {
                tempEntries[tempCount++] = e;
            }
        }
        portEXIT_CRITICAL(&g_logMux);

        String json = "[";
        for (size_t i = 0; i < tempCount; i++) {
            if (i > 0) json += ",";
            char escTag[32]; escapeJsonString(tempEntries[i].tag, escTag, sizeof(escTag));
            char escMsg[240]; escapeJsonString(tempEntries[i].msg, escMsg, sizeof(escMsg));
            json += "{\"id\":" + String(tempEntries[i].id) + ",\"ts\":" + String(tempEntries[i].ts) +
                    ",\"tag\":\"" + String(escTag) + "\",\"msg\":\"" + String(escMsg) + "\"}";
        }
        json += "]";
        request->send(200, "application/json", json);
    });

    // API: Clear Logs
    server.on("/api/logs/clear", HTTP_POST, [](AsyncWebServerRequest* request) {
        enqueueCommand(CMD_CLEAR_LOGS);
        request->send(200, "text/plain", "CLEARED");
    });

    // API: Betaflight TCP Bridge Toggle
    server.on("/api/bf_toggle", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (g_bridgeMode) {
            enqueueCommand(CMD_EXIT_TCP_BRIDGE);
        } else {
            enqueueCommand(CMD_ENTER_TCP_BRIDGE);
        }
        request->send(200, "text/plain", "OK");
    });

    // Fallback Captive Portal Redirect
    server.onNotFound([](AsyncWebServerRequest* request) {
        String host = request->host();
        if (host == "192.168.4.1" || host.startsWith("192.168.4.")) {
            request->send(404, "text/plain", "Not Found");
        } else {
            request->redirect("http://192.168.4.1/");
        }
    });

    // Attach WebSocket handler to HTTP server
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.begin();
    addLog("HTTP", "AsyncWebServer and Captive Portal active on port 80");
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    addLog("BOOT", "OpenLogic Multi-Camera Controller Starting...");

    // Create FreeRTOS Command Queue (depth 16)
    g_cmdQueue = xQueueCreate(16, sizeof(CommandItem));

    // Initialize NVS
    prefs.begin("camSwitch", false);
    g_bridgeMode = prefs.getBool("bf_mode", false);
    if (g_bridgeMode) {
        prefs.putBool("bf_mode", false); // One-shot: clear immediately so normal power-cycle returns to camera mode
    }

    g_autoRecOnArm = prefs.getBool("auto_arm", true);
    g_disarmDelaySec = prefs.getUChar("disarm_dly", 3);
    if (g_disarmDelaySec > 15) g_disarmDelaySec = 3;
    g_auxRecChannel = prefs.getUChar("aux_ch", 0);
    if (g_auxRecChannel > 8) g_auxRecChannel = 0;
    g_wifiOffOnArm = prefs.getBool("wifi_arm", false);

    g_activeBrand = (CameraBrand)prefs.getUChar("brand", (uint8_t)BRAND_GOPRO);
    if (g_activeBrand >= BRAND_COUNT) g_activeBrand = BRAND_GOPRO;
    g_activeDriver = g_drivers[g_activeBrand];

    // Load saved MAC for active brand
    hasSavedMacForActiveBrand = loadSavedMacForBrand(g_activeBrand, targetAddress);
    if (hasSavedMacForActiveBrand) {
        hasTargetAddress = true;
    }

    char bootMsg[64];
    snprintf(bootMsg, sizeof(bootMsg), "Boot Default Camera: %s", g_activeDriver->getBrandName());
    addLog("BOOT", bootMsg);

    // Initialize SoftAP & Captive Portal DNS
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);

    IPAddress apIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, gateway, subnet);

    WiFi.softAP(AP_SSID, AP_PASS, 6);
    delay(50);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_13dBm);

    addLog("WIFI", ("SoftAP SSID: " + String(AP_SSID) + " (WPA2) IP: " + WiFi.softAPIP().toString()).c_str());

    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);

    dnsServer.start(DNS_PORT, "*", apIP);
    addLog("DNS", "Wildcard Captive Portal DNS Server active on port 53");

    // Initialize Web Server & WebSocket
    setupWebServer();

    g_bfBridge.setLogCallback(addLog);
    g_bfBridge.begin(g_bridgeMode);

    if (g_bridgeMode) {
        addLog("BOOT", "=== DEDICATED WI-FI TCP BRIDGE MODE ACTIVE ===");
        addLog("BOOT", "Connect Betaflight Configurator to tcp://192.168.4.1:5761");
        addLog("BOOT", "BLE stack disabled for zero RF interference");
    } else {
        // Initialize NimBLE Stack strictly for active brand role
        if (g_activeBrand == BRAND_INSTA360) {
            // BLE Peripheral named "Insta360 GPS Remote"
            NimBLEDevice::init("Insta360 GPS Remote");
            NimBLEDevice::setMTU(247);

            pServer = NimBLEDevice::createServer();
            pServer->setCallbacks(new UnifiedServerCallbacks());

            NimBLEService* pSvc = pServer->createService(NimBLEUUID((uint16_t)0xCE80));
            pInstaWriteChar = pSvc->createCharacteristic(
                NimBLEUUID((uint16_t)0xCE81),
                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
            );
            pInstaWriteChar->setCallbacks(new InstaCharCallbacks());

            pInstaNotifyChar = pSvc->createCharacteristic(
                NimBLEUUID((uint16_t)0xCE82),
                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
            );
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            pSvc->start();
#pragma GCC diagnostic pop
            pServer->start();

            NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
            pAdv->setName("Insta360 GPS Remote");
            pAdv->addServiceUUID(NimBLEUUID((uint16_t)0xCE80));
            pAdv->setMinInterval(160);
            pAdv->setMaxInterval(320);
            pAdv->start();
            addLog("BLE", "Insta360 GPS Remote Peripheral active & advertising 0xCE80");
        } else {
            // Pure Central Mode for DJI & GoPro (No BLE Server / No NUS / No SpeedyBee BLE!)
            NimBLEDevice::init(BLE_DEVICE_NAME);
            NimBLEDevice::setMTU(247);

            pClient = NimBLEDevice::createClient();
            pClient->setClientCallbacks(new ClientCallbacks(), false);

            NimBLEScan* pScan = NimBLEDevice::getScan();
            pScan->setScanCallbacks(new ScanCallbacks(), false);
            startBleScan();
            addLog("BLE", "NimBLE Central initialized & scanning...");
        }
    }
}

// ============================================================================
// Arm & AUX Edge-Triggered Recording Controller (loopTask)
// ============================================================================
static void runArmAndAuxController(uint32_t now) {
    bool fcConnected = g_bfBridge.isFcConnected();
    bool isArmed = g_bfBridge.isArmed();

    static bool s_lastArmed = false;
    static bool s_auxHigh = false;
    static uint32_t s_lastMspSampleMs = 0;
    static uint8_t s_armCandidateCount = 0;
    static bool s_lastCandidateArmed = false;

    bool camConnected = false;
    if (g_activeBrand == BRAND_INSTA360) {
        camConnected = g_instaConnected;
    } else {
        camConnected = (pClient && pClient->isConnected());
    }

    // 1. Arm Trigger (sampled on each MSP packet, debounced over 2 consecutive reads)
    if (fcConnected) {
        uint32_t mspRespMs = g_bfBridge.getLastMspResponseMs();
        if (mspRespMs != s_lastMspSampleMs) {
            s_lastMspSampleMs = mspRespMs;

            if (isArmed == s_lastCandidateArmed) {
                if (s_armCandidateCount < 255) s_armCandidateCount++;
            } else {
                s_lastCandidateArmed = isArmed;
                s_armCandidateCount = 1;
            }

            if (s_armCandidateCount >= 2 && s_lastArmed != isArmed) {
                s_lastArmed = isArmed;

                if (s_lastArmed) {
                    // Disarmed -> Armed edge
                    s_disarmTargetMs = 0; // Cancel pending stop immediately
                    if (g_autoRecOnArm && camConnected) {
                        bool isRec = g_activeDriver ? g_activeDriver->getTelemetry().isRecording : false;
                        if ((g_activeBrand == BRAND_INSTA360 && !s_autoRecStarted) ||
                            ((g_activeBrand == BRAND_DJI || g_activeBrand == BRAND_GOPRO) && !isRec)) {
                            enqueueCommand(CMD_RECORD_START);
                            s_autoRecStarted = true;
                            addLog("ARM", "FC Armed -> Start Recording");
                        }
                    }
                } else {
                    // Armed -> Disarmed edge
                    if (g_autoRecOnArm && s_autoRecStarted) {
                        s_disarmTargetMs = millis() + ((uint32_t)g_disarmDelaySec * 1000);
                        char buf[64];
                        snprintf(buf, sizeof(buf), "FC Disarmed -> Stop in %u s", (unsigned)g_disarmDelaySec);
                        addLog("ARM", buf);
                    }
                }
            }
        }
    }
    // Note: Do NOT reset s_lastArmed when fcConnected drops; retain state across short timeouts

    // 2. Pending Disarm Delay Timeout
    if (s_disarmTargetMs > 0 && (int32_t)(now - s_disarmTargetMs) >= 0) {
        s_disarmTargetMs = 0;
        s_autoRecStarted = false;
        if (camConnected) {
            enqueueCommand(CMD_RECORD_STOP);
            addLog("ARM", "Disarm delay expired -> Stop Recording");
        }
    }

    // 3. AUX Channel Trigger (works even when disarmed for bench testing)
    if (fcConnected && g_auxRecChannel >= 1 && g_auxRecChannel <= 8) {
        uint16_t val = g_bfBridge.getAuxChannel(g_auxRecChannel);
        if (val >= 800) { // Ignore uninitialized channels (< 800)
            if (val >= 1700 && !s_auxHigh) {
                s_auxHigh = true;
                if (camConnected) {
                    bool isRec = g_activeDriver ? g_activeDriver->getTelemetry().isRecording : false;
                    if (!isRec) {
                        enqueueCommand(CMD_RECORD_START);
                        s_autoRecStarted = true;
                        addLog("AUX", "AUX switch HIGH -> Start Recording");
                    }
                }
            } else if (val <= 1300 && s_auxHigh) {
                s_auxHigh = false;
                if (camConnected) {
                    bool isRec = g_activeDriver ? g_activeDriver->getTelemetry().isRecording : false;
                    if (isRec || s_autoRecStarted) {
                        enqueueCommand(CMD_RECORD_STOP);
                        s_autoRecStarted = false;
                        addLog("AUX", "AUX switch LOW -> Stop Recording");
                    }
                }
            }
        }
    }
}

// ============================================================================
// Wi-Fi Power Management (Wi-Fi Off When Armed)
// ============================================================================
static void runWifiPowerManagement(uint32_t now) {
    bool isArmed = g_bfBridge.isFcConnected() && g_bfBridge.isArmed();

    if (isArmed && !g_wifiSuspended) {
        g_wifiSuspended = true;
        g_wifiRestoreAtMs = 0;
        ws.closeAll();
        dnsServer.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        addLog("WIFI", "SoftAP disabled while armed");
    } else if (!isArmed && g_wifiSuspended) {
        if (g_wifiRestoreAtMs == 0) {
            g_wifiRestoreAtMs = now + 2000;
        } else if ((int32_t)(now - g_wifiRestoreAtMs) >= 0) {
            g_wifiRestoreAtMs = 0;
            WiFi.mode(WIFI_AP);
            IPAddress apIP(192, 168, 4, 1);
            IPAddress gateway(192, 168, 4, 1);
            IPAddress subnet(255, 255, 255, 0);
            WiFi.softAPConfig(apIP, gateway, subnet);
            WiFi.softAP(AP_SSID, AP_PASS, 6);
            delay(50);
            WiFi.setSleep(false);
            WiFi.setTxPower(WIFI_POWER_13dBm);
            dnsServer.start(DNS_PORT, "*", apIP);
            g_wifiSuspended = false;
            addLog("WIFI", "SoftAP restored after landing");
        }
    }
}

// ============================================================================
// Main Loop (Runs on loopTask)
// ============================================================================
void loop() {
    uint32_t now = millis();

    // 1. Process Captive Portal DNS Requests
    if (!g_wifiSuspended) {
        for (int i = 0; i < 4; i++) {
            dnsServer.processNextRequest();
        }
    }

    // 2. Scheduled Reboot for Clean BLE Role Switching
    if (g_pendingRestart && (int32_t)(now - g_restartTargetMs) >= 0) {
        ESP.restart();
    }

    // 3. Deferred BLE Connect Processing for Insta360 on loopTask
    if (!g_bridgeMode && g_instaConnectPending) {
        g_instaConnectPending = false;
        if (g_instaConnected) {
            g_instaConnHandle = g_stagedInstaConnHandle;
            saveActiveBrandMac(g_stagedInstaAddr);
            g_activeDriver->onConnected(g_instaConnHandle, bleWriteCallback);
            portENTER_CRITICAL(&g_telemetryMux);
            g_telemetryVersion++;
            portEXIT_CRITICAL(&g_telemetryMux);
        }
    }

    // 4. Deferred BLE Disconnect Processing on loopTask
    if (!g_bridgeMode && g_disconnectPending) {
        g_disconnectPending = false;
        addLog("BLE", "Link disconnected; cleaning up on loopTask");
        connStep = CONN_IDLE;
        pWriteChar = nullptr;
        pNotifyChar = nullptr;
        pGoproSettingsWriteChar = nullptr;
        pGoproSettingsNotifyChar = nullptr;
        pGoproQueryWriteChar = nullptr;
        pGoproQueryNotifyChar = nullptr;
        g_activeDriver->onDisconnected();
        g_telemetryVersion++;

        if (g_activeBrand != BRAND_INSTA360) {
            startBleScan();
        } else {
            NimBLEDevice::getAdvertising()->start();
        }
    }

    // 5. Periodic WebSocket client cleanup
    static uint32_t lastWsCleanupMs = 0;
    if (!g_wifiSuspended && now - lastWsCleanupMs >= 500) {
        lastWsCleanupMs = now;
        ws.cleanupClients();
    }

    // 6. Drain and process Command Queue on loopTask
    if (g_cmdQueue) {
        CommandItem cmd;
        while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
            processCommand(cmd);
        }
    }

    // 7. Handle WKP Wake Broadcast Timeout (2.5s)
    if (!g_bridgeMode && isBroadcastingWake) {
        if (now - wakeBroadcastStartMs >= 2500) {
            stopWkpWakeBroadcast();
        }
        g_bfBridge.loop();
        delay(5);
        return;
    }

    // 8. Run Non-Blocking Connection State Machine (Central mode)
    if (!g_bridgeMode && g_activeBrand != BRAND_INSTA360 && connStep != CONN_IDLE) {
        runConnectionStateMachine(now);
    }

    // 9. Periodic Driver Updates (Telemetry watchdogs, heartbeats)
    if (!g_bridgeMode && g_activeDriver) {
        g_activeDriver->updateLoop(now);

        // Check if driver requested link teardown
        if (g_activeDriver->shouldDisconnect()) {
            if (pClient && pClient->isConnected()) {
                addLog("BLE", "Camera inactive / sleeping -> Teardown stale link");
                pClient->disconnect();
            }
        }
    }

    // 10. Flush pending logs to WebSocket clients
    if (!g_wifiSuspended) {
        flushLogsToWebSocket();
    }

    // 11. Rate-limited WebSocket Telemetry Broadcast (Max 5 Hz = every >= 200ms)
    static uint32_t lastWsStatePushMs = 0;
    if (!g_wifiSuspended && now - lastWsStatePushMs >= 200) {
        if (hasStateChanged()) {
            broadcastStateJson();
            lastWsStatePushMs = now;
        }
    }

    // 12. Time-sliced BLE scanning (Central mode only)
    if (!g_bridgeMode && g_activeBrand != BRAND_INSTA360) {
        if ((!pClient || !pClient->isConnected()) && connStep == CONN_IDLE && !isBroadcastingWake) {
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (!pScan->isScanning() && (now - lastScanRestartMs >= 4000)) {
                startBleScan();
            }
        }
    }

    // 13. Run Betaflight Bridge (TCP bridge or UART MSP monitor)
    g_bfBridge.loop();

    // 14. Edge-Triggered Recording Controller
    if (!g_bridgeMode) {
        runArmAndAuxController(now);
    }

    // 15. Wi-Fi Power Management (Wi-Fi Off When Armed)
    if (!g_bridgeMode && g_wifiOffOnArm) {
        runWifiPowerManagement(now);
    }

    delay(2);
}
