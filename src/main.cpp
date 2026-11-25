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

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 8
#endif

#ifndef STATUS_LED_ACTIVE_LOW
#define STATUS_LED_ACTIVE_LOW 1
#endif

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
    CMD_START_FLIGHT_MODE,
    CMD_ENTER_BF_BLE,
    CMD_SET_AUTO_ARM,
    CMD_SET_DISARM_DELAY,
    CMD_SET_AUX_CHANNEL
};

struct CommandItem {
    CmdType type;
    uint8_t value;
    uint16_t extra;
};

static QueueHandle_t g_cmdQueue = nullptr;
static uint32_t s_disarmTargetMs = 0;
static bool s_armRecStarted = false;
static bool s_auxRecStarted = false;

// Persistent Settings & Operating Mode
enum BootMode : uint8_t {
    MODE_FLIGHT = 0,
    MODE_WIFI_SETUP = 1,
    MODE_BF_BLE_BRIDGE = 2
};
static BootMode g_bootMode = MODE_FLIGHT;
static uint32_t g_bootMs = 0;
static bool g_autoWifiCancelled = false;
static bool g_autoRecOnArm = true;
static uint8_t g_disarmDelaySec = 3;
static uint8_t g_auxRecChannel = 0;

// Betaflight BLE NUS Handles & State
static NimBLECharacteristic* pNusTxChar = nullptr;
static NimBLECharacteristic* pNusRxChar = nullptr;
static volatile bool g_bfBleConnected = false;
static uint16_t g_bfBleConnHandle = 0;
static uint16_t g_bfBleMtu = 23;

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

// Deferred MAC Save State
static bool s_macSavedThisSession = false;
static uint32_t s_instaMacSaveAtMs = 0;
static NimBLEAddress s_pendingInstaAddr;

// Reboot-to-Switch State
static bool g_pendingRestart = false;
static uint32_t g_restartTargetMs = 0;

// Host-Task Thread-Safety Flags
static volatile bool g_disconnectPending = false;
static volatile int g_lastDisconnectReason = 0;
static volatile bool g_connectFailPending = false;
static volatile int g_lastConnectFailReason = 0;
static uint32_t g_bleScanAllowedAfterMs = 0;
static volatile uint32_t g_telemetryVersion = 1;
static bool g_explicitSleepRequested = false;
static bool g_cameraAsleep = false;
static bool g_pendingRecordOnWake = false;
static uint32_t g_pendingRecordExpireMs = 0;
static bool s_hadDisconnectThisBoot = false;
static bool s_diagPairedLoggedThisSession = false;
static uint32_t g_camPairedAtMs = 0;
static uint32_t g_lastDisconnectMs = 0;
static uint32_t s_cameraConnBackoffUntilMs = 0;
static bool s_expectingShutdownDisconnect = false;

// 6-Event Diagnostics Trail (Flushed to NVS on disconnect)
static char s_diagLines[6][64];
static uint8_t s_diagCount = 0;

static void flushDiagHistoryToNvs() {
    if (s_diagCount > 0 && g_bootMode == MODE_FLIGHT) {
        String joined = "";
        for (uint8_t i = 0; i < s_diagCount; i++) {
            if (i > 0) joined += "\n";
            joined += s_diagLines[i];
        }
        prefs.putString("diag_hist", joined);
    }
}

static void appendDiagEvent(const char* line, bool flushToNvs) {
    if (!line || line[0] == '\0') return;

    if (s_diagCount < 6) {
        strncpy(s_diagLines[s_diagCount], line, sizeof(s_diagLines[0]) - 1);
        s_diagLines[s_diagCount][sizeof(s_diagLines[0]) - 1] = '\0';
        s_diagCount++;
    } else {
        for (uint8_t i = 0; i < 5; i++) {
            memcpy(s_diagLines[i], s_diagLines[i + 1], sizeof(s_diagLines[0]));
        }
        strncpy(s_diagLines[5], line, sizeof(s_diagLines[0]) - 1);
        s_diagLines[5][sizeof(s_diagLines[0]) - 1] = '\0';
    }

    if (flushToNvs) {
        flushDiagHistoryToNvs();
    }
}

// WKP Wake Broadcast State
static bool isBroadcastingWake = false;
static uint32_t wakeBroadcastStartMs = 0;
static bool s_wkpSwappedToMsb = false;
static bool s_wkpScanStarted = false;
static uint8_t s_wkpMsbMac[6] = {0};

// Non-Blocking Connection State Machine
enum ConnStep {
    CONN_IDLE,
    CONN_BOOT_WAIT,
    CONN_INITIATE,
    CONN_AWAIT_CONNECT,
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

    // Mirror to USB Serial (115200 baud) for live terminal diagnostics
    if (Serial && Serial.availableForWrite() >= 96) {
        Serial.printf("[%lu][%s] %s\r\n", (unsigned long)now, tag ? tag : "SYS", msg ? msg : "");
    }
}

// Forward Declarations
static void startBleScan();
static void selectCameraBrand(CameraBrand brand, bool reboot = false);
static void startWkpWakeBroadcast();
static void stopWkpWakeBroadcast();
static void handleWkpWakeLoop(uint32_t now);
static void configureNusAdvertising();
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
    char typeKey[20];
    snprintf(typeKey, sizeof(typeKey), "%s_t", key);

    uint8_t existingMac[6] = {0};
    size_t existingLen = prefs.getBytes(key, existingMac, 6);
    uint8_t existingType = prefs.getUChar(typeKey, 0xFF);

    if (existingLen == 6 && memcmp(existingMac, msbMac, 6) == 0 && existingType == addr.getType()) {
        hasSavedMacForActiveBrand = true;
        return;
    }

    prefs.putBytes(key, msbMac, 6);
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
    g_cameraAsleep = false;
    uint8_t zeroMac[6] = {0};

    portENTER_CRITICAL(&g_telemetryMux);
    g_activeDriver->setSavedMac(zeroMac);
    g_telemetryVersion++;
    portEXIT_CRITICAL(&g_telemetryMux);

    addLog("NVS", ("Cleared saved MAC for " + String(g_activeDriver->getBrandName())).c_str());

    if (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360) {
        if (pClient && pClient->isConnected()) {
            pClient->disconnect();
        }
        startBleScan();
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

    void onConnectFail(NimBLEClient* client, int reason) override {
        g_lastConnectFailReason = reason;
        g_connectFailPending = true;
    }

    void onDisconnect(NimBLEClient* client, int reason) override {
        // Defer state mutation and scan restart to loopTask
        g_lastDisconnectReason = reason;
        g_disconnectPending = true;
    }

    bool onConnParamsUpdateRequest(NimBLEClient* client, const ble_gap_upd_params* params) override {
        if (params && params->supervision_timeout < 500) {
            return false;
        }
        return true;
    }
};

static ClientCallbacks s_clientCallbacks;

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
            pSvc->getCharacteristics(true); // Cache all characteristics in a single ATT round-trip
            pNotifyChar = pSvc->getCharacteristic(NimBLEUUID(static_cast<uint16_t>(0xFFF4)));
            pWriteChar  = pSvc->getCharacteristic(NimBLEUUID(static_cast<uint16_t>(0xFFF5)));
        }
        return (pSvc && pWriteChar && pNotifyChar);
    } else if (g_activeBrand == BRAND_GOPRO) {
        NimBLEUUID goproSvc(static_cast<uint16_t>(0xFEA6));
        NimBLERemoteService* pSvc = pClient->getService(goproSvc);
        if (pSvc) {
            pSvc->getCharacteristics(true); // Cache all characteristics in a single ATT round-trip
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
    void handleDiscoveredDevice(const NimBLEAdvertisedDevice* advertisedDevice) {
        if (!advertisedDevice || connStep != CONN_IDLE || (pClient && pClient->isConnected())) return;

        uint32_t now = millis();
        NimBLEAddress addr = advertisedDevice->getAddress();
        bool isSavedMatch = hasSavedMacForActiveBrand && hasTargetAddress &&
                            (addr == targetAddress || memcmp(addr.getVal(), targetAddress.getVal(), 6) == 0);

        bool hasFullName = advertisedDevice->haveName() && !advertisedDevice->getName().empty();
        bool hasShortName = advertisedDevice->haveType(BLE_HS_ADV_TYPE_INCOMP_NAME);
        bool hasTargetUuid = advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xFFF0)) ||
                             advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xFEA6));
        bool isAwakeAdv = hasFullName || hasShortName || hasTargetUuid;

        if (isSavedMatch) {
            std::string mfg = advertisedDevice->getManufacturerData();
            const uint8_t* mfgPtr = reinterpret_cast<const uint8_t*>(mfg.data());
            size_t mfgLen = mfg.length();
            uint8_t advType = advertisedDevice->getAdvType();
            bool isConn = advertisedDevice->isConnectable();
            int32_t bo = (int32_t)(s_cameraConnBackoffUntilMs - now);

            static uint32_t s_lastMacLogMs = 0;
            static uint8_t s_lastAdvType = 0xFF;
            static bool s_lastConn = false;
            static bool s_lastAwake = false;
            static size_t s_lastMfgLen = 0;
            static uint8_t s_lastMfgHead[16] = {0};

            size_t cmpLen = (mfgLen < sizeof(s_lastMfgHead)) ? mfgLen : sizeof(s_lastMfgHead);
            bool changed = (advType != s_lastAdvType) || (isConn != s_lastConn) ||
                           (isAwakeAdv != s_lastAwake) || (mfgLen != s_lastMfgLen) ||
                           (cmpLen > 0 && memcmp(mfgPtr, s_lastMfgHead, cmpLen) != 0);

            if (changed || (now - s_lastMacLogMs >= 500UL)) {
                s_lastMacLogMs = now;
                s_lastAdvType = advType;
                s_lastConn = isConn;
                s_lastAwake = isAwakeAdv;
                s_lastMfgLen = mfgLen;
                if (cmpLen > 0) memcpy(s_lastMfgHead, mfgPtr, cmpLen);

                char mfgHex[52] = "--";
                if (cmpLen > 0) {
                    size_t pos = 0;
                    for (size_t i = 0; i < cmpLen && pos + 3 < sizeof(mfgHex); i++) {
                        pos += snprintf(mfgHex + pos, sizeof(mfgHex) - pos, "%s%02X", (i > 0 ? " " : ""), mfgPtr[i]);
                    }
                }
                std::string nameStr = hasFullName ? advertisedDevice->getName() :
                                      (hasShortName ? advertisedDevice->getPayloadByType(BLE_HS_ADV_TYPE_INCOMP_NAME) : "");
                if (Serial && Serial.availableForWrite() >= 96) {
                    Serial.printf("[%lu][SCAN-MAC] type=%u conn=%d awake=%d name=\"%s\" uuid=%d bo=%ldms rssi=%d mfg(%u)=[%s]\r\n",
                                  (unsigned long)now, (unsigned)advType, isConn ? 1 : 0, isAwakeAdv ? 1 : 0,
                                  nameStr.c_str(), hasTargetUuid ? 1 : 0,
                                  (long)(bo > 0 ? bo : 0), advertisedDevice->getRSSI(),
                                  (unsigned)mfgLen, mfgHex);
                }
            }
        }

        if ((int32_t)(now - s_cameraConnBackoffUntilMs) < 0) return;

        // Ignore raw standby/manufacturer beacons from sleeping cameras.
        // Only initiate connection if the camera is actively broadcasting its name or service.
        if (!isAwakeAdv) {
            return;
        }

        // 1. If saved MAC exists for active brand, prioritize matching that MAC immediately
        if (hasSavedMacForActiveBrand && hasTargetAddress) {
            if (isSavedMatch) {
                targetAddress = addr;
                char buf[64];
                snprintf(buf, sizeof(buf), "Matched saved %s [%s]", g_activeDriver->getBrandName(), addr.toString().c_str());
                addLog("SCAN", buf);
                char diagEvt[32];
                snprintf(diagEvt, sizeof(diagEvt), "S@%lu", (unsigned long)now);
                appendDiagEvent(diagEvt, false);
                connStateTimerMs = now;
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

            targetAddress = addr;
            hasTargetAddress = true;
            char diagEvt[32];
            snprintf(diagEvt, sizeof(diagEvt), "S@%lu", (unsigned long)now);
            appendDiagEvent(diagEvt, false);
            connStateTimerMs = now;
            connStep = CONN_INITIATE;
            portENTER_CRITICAL(&g_telemetryMux);
            g_telemetryVersion++;
            portEXIT_CRITICAL(&g_telemetryMux);
        }
    }

    void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override {
        handleDiscoveredDevice(advertisedDevice);
    }

    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        handleDiscoveredDevice(advertisedDevice);
    }
};

static uint32_t s_bfBleConnAtMs = 0;
static bool s_bfParamsUpdated = false;

// ============================================================================
// BLE Server Callbacks (Insta360 Peripheral Server & Betaflight NUS Server)
// ============================================================================
class UnifiedServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        uint16_t handle = connInfo.getConnHandle();
        if (g_bootMode == MODE_BF_BLE_BRIDGE || (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360)) {
            g_bfBleConnected = true;
            g_bfBleConnHandle = handle;
            g_bfBleMtu = connInfo.getMTU() > 0 ? connInfo.getMTU() : 23;
            g_bfBridge.setBleConnected(true);
            g_bfBridge.setBleMtu(g_bfBleMtu);
            g_autoWifiCancelled = true;
            s_bfBleConnAtMs = millis();
            s_bfParamsUpdated = false;
            addLog("BF_BLE", "Betaflight Configurator connected over BLE");
            g_telemetryVersion++;
        } else if (g_activeBrand == BRAND_INSTA360) {
            addLog("BLE", "Insta360 camera connected to GPS Remote");
            g_instaConnected = true;
            g_stagedInstaConnHandle = handle;
            g_stagedInstaAddr = connInfo.getAddress();
            g_instaConnectPending = true;
        }
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        uint16_t handle = connInfo.getConnHandle();
        if (g_bfBleConnected && (handle == g_bfBleConnHandle || g_bfBleConnHandle == 0)) {
            addLog("BF_BLE", "Betaflight Configurator disconnected from BLE");
            g_bfBleConnected = false;
            g_bfBleConnHandle = 0;
            g_bfBleMtu = 23;
            s_bfBleConnAtMs = 0;
            s_bfParamsUpdated = false;
            g_bfBridge.setBleConnected(false);
            g_telemetryVersion++;
            if (g_bootMode == MODE_BF_BLE_BRIDGE) {
                NimBLEDevice::getAdvertising()->start();
            }
        }
        if (g_activeBrand == BRAND_INSTA360 && handle == g_instaConnHandle) {
            addLog("BLE", "Insta360 camera disconnected");
            g_instaConnected = false;
            g_instaConnHandle = 0;
            g_disconnectPending = true;
        }
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        if (g_bfBleConnected && connInfo.getConnHandle() == g_bfBleConnHandle) {
            g_bfBleMtu = MTU;
            g_bfBridge.setBleMtu(MTU);
            char buf[48];
            snprintf(buf, sizeof(buf), "Betaflight BLE MTU updated to %u", (unsigned)MTU);
            addLog("BF_BLE", buf);
        }
    }

    void onConnParamsUpdate(NimBLEConnInfo& connInfo) override {
        if (g_bfBleConnected && connInfo.getConnHandle() == g_bfBleConnHandle) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Betaflight BLE interval negotiated to %.2f ms (latency=%u)",
                     connInfo.getConnInterval() * 1.25f, (unsigned)connInfo.getConnLatency());
            addLog("BF_BLE", buf);
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

class BfNusTxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        char buf[64];
        snprintf(buf, sizeof(buf), "Betaflight Configurator subscribed to TX (subValue=%u)", (unsigned)subValue);
        addLog("BF_BLE", buf);

        if (subValue > 0 && pServer) {
            // Request OS-compliant low-latency interval: 7.5ms min, 15ms max, 0 latency, 4s timeout
            pServer->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 400);
            s_bfParamsUpdated = true;
            addLog("BF_BLE", "Fast BLE connection interval (7.5ms-15ms) requested for zero-lag telemetry");
        }
    }
};

class BfNusRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        NimBLEAttValue val = pCharacteristic->getValue();
        if (val.length() > 0) {
            g_bfBridge.pushBleRx(val.data(), val.length());
        }
    }
};

static bool bfBleTxCallback(const uint8_t* data, size_t len) {
    if (!g_bfBleConnected || !pNusTxChar || !data || len == 0) return false;
    return pNusTxChar->notify(data, len, g_bfBleConnHandle);
}

static void recordFirstDisconnect(const char* src, int reason, uint32_t nowMs) {
    if (g_bootMode != MODE_FLIGHT) return;

    CameraTelemetry t{};
    if (g_activeDriver) {
        portENTER_CRITICAL(&g_telemetryMux);
        t = g_activeDriver->getTelemetry();
        portEXIT_CRITICAL(&g_telemetryMux);
    }
    uint8_t pm = (g_activeBrand == BRAND_DJI) ? g_djiDriver.getLastPowerMode() : 0;
    uint8_t cs = (g_activeBrand == BRAND_DJI) ? g_djiDriver.getLastCamStatus() : 0;

    char diagEvt[64];
    snprintf(diagEvt, sizeof(diagEvt), "D@%lu %s r=0x%X st=%u pm=%u cs=%u pkt=%lu",
             (unsigned long)nowMs, src ? src : "UNK", (unsigned)reason, (unsigned)t.state,
             (unsigned)pm, (unsigned)cs, (unsigned long)t.packetCount);
    appendDiagEvent(diagEvt, true);
}

static const char* const NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";

static void configureNusAdvertising() {
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (!pAdv) return;

    if (pAdv->isAdvertising()) {
        pAdv->stop();
    }
    pAdv->reset();
    pAdv->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    pAdv->enableScanResponse(true);

    NimBLEAdvertisementData advData;
    advData.setFlags(0x06);
    advData.setCompleteServices(NimBLEUUID(NUS_SERVICE_UUID));

    NimBLEAdvertisementData scanResp;
    scanResp.setName("OpenLogic_BF");

    pAdv->setAdvertisementData(advData);
    pAdv->setScanResponseData(scanResp);
    pAdv->setMinInterval(160);
    pAdv->setMaxInterval(240);
}

static bool s_nusServerStarted = false;

static void setupBetaflightBleServer(bool startAdv = true) {
    if (!s_nusServerStarted) {
        s_nusServerStarted = true;
        if (!pServer) {
            pServer = NimBLEDevice::createServer();
            pServer->setCallbacks(new UnifiedServerCallbacks());
        }

        NimBLEService* pNusService = pServer->createService(NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"));

        pNusTxChar = pNusService->createCharacteristic(
            NimBLEUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        pNusTxChar->setCallbacks(new BfNusTxCallbacks());

        pNusRxChar = pNusService->createCharacteristic(
            NimBLEUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
        );
        pNusRxChar->setCallbacks(new BfNusRxCallbacks());

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        pNusService->start();
#pragma GCC diagnostic pop

        pServer->start();

        g_bfBridge.setBleTxCallback(bfBleTxCallback);

        configureNusAdvertising();
    }

    if (startAdv) {
        NimBLEDevice::getAdvertising()->start();
        addLog("BF_BLE", "Betaflight NUS Server advertising 'OpenLogic_BF' (6E400001)");
    }
}

// ============================================================================
// Non-Blocking Connection State Machine (Central Mode)
// ============================================================================
static uint8_t s_resolveRetries = 0;

static void runConnectionStateMachine(uint32_t now) {
    static ConnStep s_lastLoggedConnStep = CONN_IDLE;
    if (connStep != s_lastLoggedConnStep) {
        if (Serial && Serial.availableForWrite() >= 96) {
            Serial.printf("[%lu][CONN] State transition: %d -> %d\r\n", (unsigned long)now, (int)s_lastLoggedConnStep, (int)connStep);
        }
        s_lastLoggedConnStep = connStep;
    }

    switch (connStep) {
        case CONN_BOOT_WAIT: {
            connStep = CONN_INITIATE;
            g_telemetryVersion++;
            break;
        }
        case CONN_INITIATE: {
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (pScan && pScan->isScanning()) {
                pScan->stop();
            }
            NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
            if (pAdv && pAdv->isAdvertising()) {
                pAdv->stop();
            }

            s_resolveRetries = 0;
            s_macSavedThisSession = false;
            s_diagPairedLoggedThisSession = false;
            g_explicitSleepRequested = false;
            s_expectingShutdownDisconnect = false;
            g_connectFailPending = false;
            addLog("BLE", ("Connecting to " + targetAddress.toString() + "...").c_str());

            if (!pClient) {
                pClient = NimBLEDevice::createClient();
                pClient->setClientCallbacks(&s_clientCallbacks, false);
            }
            if (!pClient) {
                addLog("BLE", "Failed to create BLE Client");
                s_cameraConnBackoffUntilMs = now + 800UL;
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }

            if (g_activeBrand == BRAND_GOPRO) {
                NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
                NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
            } else {
                NimBLEDevice::setSecurityAuth(0);
            }

            // 20-40ms avoids 0x3E cold-boot establishment retries on DJI; 6.0s supervision timeout
            pClient->setConnectionParams(16, 32, 0, 600);
#if CONFIG_BT_NIMBLE_EXT_ADV
            pClient->setConnectPhy(BLE_GAP_LE_PHY_1M_MASK);
#endif

            connStateTimerMs = millis();
            connStep = CONN_AWAIT_CONNECT;
            g_telemetryVersion++;

            if (!pClient->connect(targetAddress, true, true, true)) {
                addLog("BLE", "GATT connect initiate failed");
                s_cameraConnBackoffUntilMs = millis() + 800UL;
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }
            break;
        }

        case CONN_AWAIT_CONNECT: {
            if (pClient->isConnected()) {
                connStateTimerMs = now;
                connStep = CONN_AWAIT_SETTLE;
                g_telemetryVersion++;
            } else if (g_connectFailPending) {
                g_connectFailPending = false;
                char failMsg[64];
                snprintf(failMsg, sizeof(failMsg), "GATT connect failed (reason=0x%02X)", (unsigned)g_lastConnectFailReason);
                addLog("BLE", failMsg);
                s_cameraConnBackoffUntilMs = now + 200UL;
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
            } else if ((int32_t)(now - connStateTimerMs) >= 5000) {
                addLog("BLE", "GATT connect timed out (5000ms)");
                pClient->cancelConnect();
                if (pClient->isConnected()) {
                    pClient->disconnect();
                }
                s_cameraConnBackoffUntilMs = now + 800UL;
                char diagEvt[32];
                snprintf(diagEvt, sizeof(diagEvt), "T@%lu", (unsigned long)now);
                appendDiagEvent(diagEvt, false);
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
            }
            break;
        }

        case CONN_AWAIT_SETTLE: {
            uint32_t settleDelay = (g_activeBrand == BRAND_GOPRO) ? 150 : 20;
            if (now - connStateTimerMs >= settleDelay) {
                if (!pClient->isConnected()) {
                    addLog("BLE", "Link dropped during connection settle");
                    s_cameraConnBackoffUntilMs = now + 800UL;
                    connStep = CONN_IDLE;
                    g_telemetryVersion++;
                    startBleScan();
                    break;
                }
                if (g_activeBrand == BRAND_GOPRO) {
                    addLog("BLE", "Securing connection for GoPro bond/encryption...");
                    if (!pClient->secureConnection()) {
                        if (s_resolveRetries < 1) {
                            s_resolveRetries++;
                            connStateTimerMs = now;
                            break;
                        }
                        addLog("BLE", "GoPro secureConnection failed");
                        recordFirstDisconnect("GOPRO_SEC", 0x216, now);
                        s_cameraConnBackoffUntilMs = now + 800UL;
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
                s_cameraConnBackoffUntilMs = now + 800UL;
                connStep = CONN_IDLE;
                g_telemetryVersion++;
                startBleScan();
                break;
            }
            if (!resolveCameraCharacteristics()) {
                if (s_resolveRetries < 6) {
                    s_resolveRetries++;
                    connStateTimerMs = now;
                    connStep = CONN_AWAIT_SETTLE;
                    break;
                }
                addLog("BLE", "Required GATT Characteristics not found");
                recordFirstDisconnect("RESOLVE_CHAR", 0x216, now);
                s_cameraConnBackoffUntilMs = now + 800UL;
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
            uint32_t cccdSettle = (g_activeBrand == BRAND_GOPRO) ? 60 : 10;
            if ((int32_t)(now - connStateTimerMs) >= (int32_t)cccdSettle) {
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
            if (pClient->getMTU() < 64) {
                pClient->exchangeMTU();
                delay(50);
            }
            // Never execute BLE writes / handshakes inside spinlocks!
            g_activeDriver->onConnected(pClient->getConnHandle(), bleWriteCallback);
            addLog("BLE", "Camera initialized and authenticated");

            char diagEvt[64];
            snprintf(diagEvt, sizeof(diagEvt), "C@%lu", (unsigned long)now);
            appendDiagEvent(diagEvt, false);

            connStep = CONN_IDLE;
            g_telemetryVersion++;
            break;
        }

        default:
            break;
    }
}

static void startBleScan() {
    if (g_bootMode != MODE_FLIGHT || g_bfBleConnected || g_activeBrand == BRAND_INSTA360) return;
    if (isBroadcastingWake || connStep != CONN_IDLE) return;
    if (pClient && pClient->isConnected()) return;
    if ((int32_t)(millis() - g_bleScanAllowedAfterMs) < 0) return;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true); // Active scan so cameras advertising name/UUID in Scan Response are discovered
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (pAdv && pAdv->isAdvertising()) {
        pScan->setInterval(40);  // 40 * 0.625ms = 25ms interval
        pScan->setWindow(32);    // 80% duty cycle: fast discovery while coexisting with NUS adv
    } else {
        pScan->setInterval(32);  // 32 * 0.625ms = 20ms interval
        pScan->setWindow(28);    // 87.5% duty cycle for fast camera connect/reconnect
    }
    pScan->setMaxResults(16);
    pScan->start(600, false);    // 600ms window refreshes duplicate cache rapidly for booting cameras
    lastScanRestartMs = millis();
}

static void selectCameraBrand(CameraBrand brand, bool reboot) {
    if (brand >= BRAND_COUNT) return;

    g_activeBrand = brand;
    g_activeDriver = g_drivers[brand];

    // Load saved MAC if any for this newly selected brand
    hasSavedMacForActiveBrand = loadSavedMacForBrand(g_activeBrand, targetAddress);
    hasTargetAddress = hasSavedMacForActiveBrand;

    prefs.putUChar("brand", (uint8_t)brand);
    char buf[80];
    snprintf(buf, sizeof(buf), "Switched brand to %s (saved to NVS)", g_drivers[brand]->getBrandName());
    addLog("NVS", buf);

    portENTER_CRITICAL(&g_telemetryMux);
    g_telemetryVersion++;
    portEXIT_CRITICAL(&g_telemetryMux);

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
    if (isBroadcastingWake) return;

    if (g_bootMode == MODE_WIFI_SETUP) {
        addLog("WAKE", "BLE is disabled in Wi-Fi Setup Mode to prevent RF interference. Use Flight Mode.");
        return;
    }

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
        if (prefs.getBytes("mac_dji", msbMac, 6) == 6 &&
            (msbMac[0] || msbMac[1] || msbMac[2] || msbMac[3] || msbMac[4] || msbMac[5])) {
            macFound = true;
            if (g_activeBrand != BRAND_DJI) {
                selectCameraBrand(BRAND_DJI, false);
            }
        }
    }

    if (!macFound) {
        addLog("WAKE", "No saved camera MAC found; pair camera once before WKP wake");
        if (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360) {
            startBleScan();
        }
        return;
    }

    if (!hasTargetAddress && macFound) {
        char typeKey[20];
        snprintf(typeKey, sizeof(typeKey), "%s_t", getBrandMacKey(g_activeBrand));
        uint8_t addrType = prefs.getUChar(typeKey, BLE_ADDR_PUBLIC);
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 msbMac[0], msbMac[1], msbMac[2], msbMac[3], msbMac[4], msbMac[5]);
        targetAddress = NimBLEAddress(macStr, addrType);
        hasTargetAddress = true;
        hasSavedMacForActiveBrand = true;
    }

    memcpy(s_wkpMsbMac, msbMac, 6);
    s_wkpSwappedToMsb = false;
    s_wkpScanStarted = false;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
        pScan->stop();
        delay(20);
    }

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (!pAdv) return;

    if (pAdv->isAdvertising()) {
        pAdv->stop();
        delay(20);
    }
    pAdv->reset();
    pAdv->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    pAdv->enableScanResponse(false);

    // Phase 1: Target camera MAC LSB first (reverse byte order as per DJI R-SDK)
    uint8_t wkpPayload[9];
    wkpPayload[0] = 'W';
    wkpPayload[1] = 'K';
    wkpPayload[2] = 'P';
    for (int i = 0; i < 6; i++) {
        wkpPayload[3 + i] = msbMac[5 - i]; // LSB first
    }

    NimBLEAdvertisementData advData;
    advData.setManufacturerData(wkpPayload, sizeof(wkpPayload));
    pAdv->setAdvertisementData(advData);

    pAdv->setMinInterval(32); // 20ms
    pAdv->setMaxInterval(48); // 30ms

    bool ok = pAdv->start();
    if (!ok) {
        pAdv->setMinInterval(160);
        pAdv->setMaxInterval(160);
        ok = pAdv->start();
    }
    isBroadcastingWake = true;
    wakeBroadcastStartMs = millis();

    char buf[128];
    snprintf(buf, sizeof(buf), "Broadcasting WKP wake for %02X:%02X:%02X:%02X:%02X:%02X (3.0s) [ok=%d]",
             msbMac[0], msbMac[1], msbMac[2], msbMac[3], msbMac[4], msbMac[5], ok ? 1 : 0);
    addLog("WAKE", buf);
}

static void stopWkpWakeBroadcast() {
    bool returningIdle = (connStep == CONN_IDLE);
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (pAdv) {
        if (pAdv->isAdvertising()) {
            pAdv->stop();
        }
        if (returningIdle) {
            pAdv->reset();
        }
    }
    isBroadcastingWake = false;
    s_wkpSwappedToMsb = false;
    s_wkpScanStarted = false;
    addLog("WAKE", returningIdle
           ? "Wake broadcast completed. Resuming scan..."
           : "Camera awake detected during WKP -> Connecting immediately...");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan) {
        pScan->setDuplicateFilter(true);
    }

    if (returningIdle) {
        // Restore NUS advertising payload only when returning to idle search
        if (g_bootMode == MODE_BF_BLE_BRIDGE || (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360)) {
            configureNusAdvertising();
        }

        if (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360) {
            s_cameraConnBackoffUntilMs = 0;
            g_bleScanAllowedAfterMs = 0;
            lastScanRestartMs = 0;
            startBleScan();
        }
    }
}

static void handleWkpWakeLoop(uint32_t now) {
    if (!isBroadcastingWake) return;

    // Stop WKP immediately once ScanCallbacks detects the camera's awake advertisement
    if (connStep != CONN_IDLE) {
        stopWkpWakeBroadcast();
        return;
    }

    // Refresh wall clock to prevent unsigned underflow if startWkpWakeBroadcast() ran earlier in this loop() pass
    now = millis();
    int32_t elapsed = (int32_t)(now - wakeBroadcastStartMs);
    if (elapsed < 0) return;

    // After initial 300ms pure WKP burst, scan concurrently so we catch the camera the instant it wakes
    if (!s_wkpScanStarted && (elapsed >= 300)) {
        s_wkpScanStarted = true;
        if (g_bootMode == MODE_FLIGHT && g_activeBrand != BRAND_INSTA360 && !g_bfBleConnected) {
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (pScan && !pScan->isScanning()) {
                pScan->setActiveScan(true);
                pScan->setDuplicateFilter(false); // Prevent pre-wake standby beacons from suppressing awake adv
                pScan->setInterval(48);           // 30ms scan interval
                pScan->setWindow(24);             // 15ms scan window (50% scan, 50% WKP airtime)
                pScan->setMaxResults(16);
                pScan->start(3000, false);
            }
        }
    }

    // After 2000ms of continuous LSB-first WKP (DJI R-SDK spec), alternate cleanly to MSB-first order
    if (!s_wkpSwappedToMsb && (elapsed >= 2000)) {
        s_wkpSwappedToMsb = true;
        NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
        if (pAdv) {
            uint8_t wkpPayloadMsb[9];
            wkpPayloadMsb[0] = 'W';
            wkpPayloadMsb[1] = 'K';
            wkpPayloadMsb[2] = 'P';
            for (int i = 0; i < 6; i++) {
                wkpPayloadMsb[3 + i] = s_wkpMsbMac[i]; // MSB first
            }
            if (pAdv->isAdvertising()) {
                pAdv->stop();
                delay(20);
            }
            NimBLEAdvertisementData advDataMsb;
            advDataMsb.setManufacturerData(wkpPayloadMsb, sizeof(wkpPayloadMsb));
            pAdv->setAdvertisementData(advDataMsb);
            pAdv->start();
        }
    }

    if (elapsed >= 3000) {
        stopWkpWakeBroadcast();
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

    if (g_bootMode != MODE_FLIGHT) {
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
    if (g_bootMode == MODE_WIFI_SETUP) {
        stateStr = "SETUP_MODE";
    } else if (g_bootMode == MODE_BF_BLE_BRIDGE) {
        stateStr = "BF_BLE_MODE";
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
    bool bfConn = g_bfBleConnected || g_bfBridge.isBleConnected();

    return snprintf(buf, maxLen,
        "{\"type\":\"state\",\"activeBrand\":%d,\"brandName\":\"%s\",\"stateStr\":\"%s\","
        "\"isRecording\":%s,\"battery\":%u,\"currentMode\":\"%s\","
        "\"resolutionFps\":\"%s\",\"stabilization\":\"%s\",\"tempStatus\":\"%s\","
        "\"recTime\":\"%s\",\"remTime\":\"%s\",\"remMB\":%lu,\"remPhotos\":%lu,"
        "\"mac\":\"%s\",\"packetCount\":%lu,\"lastHex\":\"%s\","
        "\"bootMode\":%u,\"bridgeMode\":%s,\"bfConnected\":%s,\"bfRxBytes\":%lu,\"bfTxBytes\":%lu,"
        "\"fcConnected\":%s,\"fcArmed\":%s,\"auxValue\":%u,\"autoRecArm\":%s,"
        "\"disarmDelaySec\":%u,\"auxRecCh\":%u}",
        (int)brand, brandName, stateStr,
        t.isRecording ? "true" : "false",
        t.batteryPercent, escMode, escFps, escEis, escTemp,
        recTimeStr, remTimeStr,
        (unsigned long)t.remainCapacityMB, (unsigned long)t.remainPhotos,
        t.cameraMac, (unsigned long)t.packetCount, escHex,
        (unsigned)g_bootMode,
        (g_bootMode == MODE_BF_BLE_BRIDGE) ? "true" : "false",
        bfConn ? "true" : "false",
        (unsigned long)g_bfBridge.getRxBytes(),
        (unsigned long)g_bfBridge.getTxBytes(),
        g_bfBridge.isFcConnected() ? "true" : "false",
        g_bfBridge.isArmed() ? "true" : "false",
        (unsigned)auxVal,
        g_autoRecOnArm ? "true" : "false",
        (unsigned)g_disarmDelaySec,
        (unsigned)g_auxRecChannel
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
    uint8_t bootMode;
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
    if (g_bootMode != MODE_FLIGHT) {
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

    bool bfConn = g_bfBleConnected || g_bfBridge.isBleConnected();
    bool fcConn = g_bfBridge.isFcConnected();
    bool fcArmed = g_bfBridge.isArmed();
    uint16_t auxVal = (g_auxRecChannel >= 1 && g_auxRecChannel <= 8) ? g_bfBridge.getAuxChannel(g_auxRecChannel) : 0;
    uint32_t bfTx = g_bfBridge.getTxBytes();
    uint32_t bfRx = g_bfBridge.getRxBytes();

    uint32_t now = millis();
    static uint32_t s_lastBytesPushMs = 0;
    bool bytesChanged = (g_bootMode == MODE_WIFI_SETUP) && ((s_lastSnapshot.bfTxBytes != bfTx || s_lastSnapshot.bfRxBytes != bfRx) && (now - s_lastBytesPushMs >= 1000));
    bool auxChanged = (abs((int)s_lastSnapshot.auxValue - (int)auxVal) > 25);

    if (s_lastSnapshot.version != g_telemetryVersion ||
        s_lastSnapshot.state != t.state ||
        s_lastSnapshot.isConnected != conn ||
        s_lastSnapshot.brand != g_activeBrand ||
        s_lastSnapshot.bootMode != (uint8_t)g_bootMode ||
        s_lastSnapshot.bfConnected != bfConn ||
        s_lastSnapshot.fcConnected != fcConn ||
        s_lastSnapshot.fcArmed != fcArmed ||
        auxChanged ||
        bytesChanged) {

        s_lastSnapshot.version = g_telemetryVersion;
        s_lastSnapshot.state = t.state;
        s_lastSnapshot.isConnected = conn;
        s_lastSnapshot.brand = g_activeBrand;
        s_lastSnapshot.bootMode = (uint8_t)g_bootMode;
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
            g_explicitSleepRequested = true;
            g_activeDriver->powerSleep();
            g_telemetryVersion++;
            addLog("CMD", "Power Sleep command sent");
            break;
        case CMD_WAKE:
            g_explicitSleepRequested = false;
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
            selectCameraBrand((CameraBrand)item.value, false);
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
        case CMD_START_FLIGHT_MODE:
            prefs.putUChar("boot_mode", (uint8_t)MODE_FLIGHT);
            addLog("SYSTEM", "Rebooting into BLE Flight Mode...");
            g_pendingRestart = true;
            g_restartTargetMs = millis() + 250;
            break;
        case CMD_ENTER_BF_BLE:
            prefs.putUChar("boot_mode", (uint8_t)MODE_BF_BLE_BRIDGE);
            addLog("SYSTEM", "Rebooting into dedicated Betaflight BLE Mode...");
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
    else if (strcmp(cmd, "start_flight") == 0) enqueueCommand(CMD_START_FLIGHT_MODE);
    else if (strcmp(cmd, "enter_bf_ble") == 0) enqueueCommand(CMD_ENTER_BF_BLE);
    else if (strcmp(cmd, "set_auto_arm") == 0) enqueueCommand(CMD_SET_AUTO_ARM, val ? atoi(val) : 1);
    else if (strcmp(cmd, "set_disarm_delay") == 0) enqueueCommand(CMD_SET_DISARM_DELAY, val ? atoi(val) : 3);
    else if (strcmp(cmd, "set_aux_ch") == 0) enqueueCommand(CMD_SET_AUX_CHANNEL, val ? atoi(val) : 0);
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

    // API: Start Flight Mode (Reboot into BLE Flight Mode)
    server.on("/api/start_flight", HTTP_POST, [](AsyncWebServerRequest* request) {
        enqueueCommand(CMD_START_FLIGHT_MODE);
        request->send(200, "text/plain", "OK");
    });

    // API: Enter Betaflight BLE Bridge Mode
    server.on("/api/enter_bf_ble", HTTP_POST, [](AsyncWebServerRequest* request) {
        enqueueCommand(CMD_ENTER_BF_BLE);
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
// Status LED Helper (Non-blocking mode indicator)
// ============================================================================
static void updateStatusLed(uint32_t now) {
    bool ledOn = false;

    if (g_bootMode == MODE_WIFI_SETUP) {
        // Mode 1: Wi-Fi Setup Mode: Fast blink (150ms toggle)
        ledOn = ((now / 150) % 2) == 0;
    } else if (g_bootMode == MODE_BF_BLE_BRIDGE) {
        // Mode 2: Dedicated Betaflight BLE Bridge: solid ON when connected; medium blink (400ms toggle) while advertising
        if (g_bfBleConnected) {
            ledOn = true;
        } else {
            ledOn = ((now / 400) % 2) == 0;
        }
    } else {
        // Mode 0: BLE Flight Mode: Camera state indicator; solid ON if Betaflight BLE connected
        if (g_bfBleConnected) {
            ledOn = true;
        } else {
            bool camConnected = (g_activeBrand == BRAND_INSTA360) ? g_instaConnected : (pClient && pClient->isConnected());

            CameraTelemetry t{};
            if (g_activeDriver) {
                portENTER_CRITICAL(&g_telemetryMux);
                t = g_activeDriver->getTelemetry();
                portEXIT_CRITICAL(&g_telemetryMux);
            }

            bool isRecording = camConnected && ((g_activeBrand == BRAND_INSTA360) ? (s_armRecStarted || s_auxRecStarted) : (t.isRecording || t.state == CAM_RECORDING));
            bool isPaired = camConnected && ((g_activeBrand == BRAND_INSTA360) || (t.state == CAM_PAIRED));

            if (isBroadcastingWake) {
                // Fast strobe: 80ms ON, 80ms OFF (160ms cycle)
                ledOn = (now % 160) < 80;
            } else if (isRecording) {
                // Recording: double-blink on 1200ms cycle
                uint32_t phase = now % 1200;
                ledOn = (phase < 100) || (phase >= 220 && phase < 320);
            } else if (isPaired) {
                // Paired & idle: solid ON
                ledOn = true;
            } else {
                // Active search / disconnected / scanning: crisp double-pulse every 1500ms
                uint32_t phase = now % 1500;
                ledOn = (phase < 80) || (phase >= 200 && phase < 280);
            }
        }
    }

    // Write the pin cleanly (avoid redundant writes if state hasn't changed)
    static int8_t s_lastLedState = -1;
    int8_t pinLevel = STATUS_LED_ACTIVE_LOW ? (ledOn ? LOW : HIGH) : (ledOn ? HIGH : LOW);
    if (pinLevel != s_lastLedState) {
        s_lastLedState = pinLevel;
        digitalWrite(STATUS_LED_PIN, pinLevel);
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(50);

    // Initialize Status LED & perform startup visual self-test (two quick 40ms flashes)
    pinMode(STATUS_LED_PIN, OUTPUT);
    for (int i = 0; i < 2; i++) {
        digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? LOW : HIGH); // ON
        delay(40);
        digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW); // OFF
        delay(40);
    }

    // Initialize BOOT button (GPIO 9) with internal pullup
    pinMode(9, INPUT_PULLUP);

    addLog("BOOT", "OpenLogic Multi-Camera Controller Starting...");

    // Create FreeRTOS Command Queue (depth 16)
    g_cmdQueue = xQueueCreate(16, sizeof(CommandItem));

    // Initialize NVS
    prefs.begin("camSwitch", false);
    g_bootMode = (BootMode)prefs.getUChar("boot_mode", (uint8_t)MODE_FLIGHT);
    if (g_bootMode != MODE_FLIGHT) {
        prefs.putUChar("boot_mode", (uint8_t)MODE_FLIGHT); // One-shot: clear immediately so normal power-cycle returns to Flight Mode
    }
    g_bootMs = millis();
    g_autoWifiCancelled = false;

    g_autoRecOnArm = prefs.getBool("auto_arm", true);
    g_disarmDelaySec = prefs.getUChar("disarm_dly", 3);
    if (g_disarmDelaySec > 15) g_disarmDelaySec = 3;
    g_auxRecChannel = prefs.getUChar("aux_ch", 0);
    if (g_auxRecChannel > 8) g_auxRecChannel = 0;

    uint8_t dfltBrand = BRAND_DJI;
    if (prefs.isKey("mac_gopro") && !prefs.isKey("mac_dji")) {
        dfltBrand = BRAND_GOPRO;
    }
    g_activeBrand = (CameraBrand)prefs.getUChar("brand", dfltBrand);
    if (g_activeBrand >= BRAND_COUNT) g_activeBrand = BRAND_DJI;
    g_activeDriver = g_drivers[g_activeBrand];

    // Load saved MAC for active brand
    hasSavedMacForActiveBrand = loadSavedMacForBrand(g_activeBrand, targetAddress);
    if (hasSavedMacForActiveBrand) {
        hasTargetAddress = true;
    }

    char bootMsg[64];
    snprintf(bootMsg, sizeof(bootMsg), "Boot Default Camera: %s", g_activeDriver->getBrandName());
    addLog("BOOT", bootMsg);

    g_bfBridge.setLogCallback(addLog);

    if (g_bootMode == MODE_WIFI_SETUP) {
        // ====================================================================
        // MODE 1: WI-FI SETUP (SoftAP & Web UI ONLY)
        // ====================================================================
        addLog("BOOT", "=== WI-FI SETUP MODE ACTIVE ===");
        addLog("BOOT", "BLE stack disabled for zero RF interference");

        String hist = prefs.getString("diag_hist", "");
        if (hist.length() > 0) {
            int startIdx = 0;
            while (startIdx < (int)hist.length()) {
                int nextIdx = hist.indexOf('\n', startIdx);
                if (nextIdx < 0) nextIdx = hist.length();
                String line = hist.substring(startIdx, nextIdx);
                line.trim();
                if (line.length() > 0) {
                    addLog("DIAG", line.c_str());
                }
                startIdx = nextIdx + 1;
            }
            prefs.remove("diag_hist");
        }
        prefs.remove("last_disc_r");
        prefs.remove("last_disc_src");
        prefs.remove("last_disc_ms");
        prefs.remove("last_disc_st");
        prefs.remove("last_disc_pkt");
        prefs.remove("last_disc_pm");
        prefs.remove("last_disc_cs");
        prefs.remove("last_disc_age");

        g_bfBridge.begin(true); // Passthrough / pause internal MSP polling

        WiFi.persistent(false);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);

        IPAddress apIP(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(apIP, gateway, subnet);
        WiFi.softAP(AP_SSID, AP_PASS, 1);
        delay(50);
        WiFi.setSleep(false);
        WiFi.setTxPower(WIFI_POWER_8_5dBm);

        addLog("WIFI", ("SoftAP SSID: " + String(AP_SSID) + " (WPA2) IP: " + WiFi.softAPIP().toString()).c_str());

        dnsServer.start(DNS_PORT, "*", apIP);
        addLog("DNS", "Wildcard Captive Portal DNS Server active on port 53");

        setupWebServer();
    } else if (g_bootMode == MODE_BF_BLE_BRIDGE) {
        // ====================================================================
        // MODE 2: DEDICATED BETAFLIGHT BLE BRIDGE MODE
        // ====================================================================
        addLog("BOOT", "=== DEDICATED BETAFLIGHT BLE BRIDGE MODE ACTIVE ===");
        addLog("BOOT", "Connect Betaflight Configurator (app.betaflight.com) via Web Bluetooth to 'OpenLogic_BF'");
        addLog("BOOT", "Wi-Fi radio turned off; Camera BLE paused");
        WiFi.persistent(false);
        WiFi.mode(WIFI_OFF);

        g_bfBridge.begin(true); // Passthrough / pause internal MSP polling

        NimBLEDevice::init("OpenLogic_BF");
        NimBLEDevice::setPower(9);
        NimBLEDevice::setMTU(247);

        setupBetaflightBleServer(true);
    } else {
        // ====================================================================
        // MODE 0: BLE FLIGHT MODE (DEFAULT ON BOOT)
        // ====================================================================
        addLog("BOOT", "=== BLE FLIGHT MODE ACTIVE ===");
        addLog("BOOT", "Wi-Fi radio turned off for zero RF contention");
        WiFi.persistent(false);
        WiFi.mode(WIFI_OFF);

        g_bfBridge.begin(false); // Normal MSP arm & AUX monitoring

        // Initialize NimBLE strictly for active brand role
        if (g_activeBrand == BRAND_INSTA360) {
            NimBLEDevice::init("Insta360 GPS Remote");
            NimBLEDevice::setPower(9);
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
            NimBLEDevice::init(BLE_DEVICE_NAME);
            NimBLEDevice::setPower(9);
            NimBLEDevice::setMTU(247);
            ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_1M_MASK);

            setupBetaflightBleServer(false);

            pClient = NimBLEDevice::createClient();
            pClient->setClientCallbacks(&s_clientCallbacks, false);

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
    static uint32_t s_lastStatusSampleMs = 0;
    static uint8_t s_armCandidateCount = 0;
    static bool s_lastCandidateArmed = false;

    bool camConnected = false;
    if (g_activeBrand == BRAND_INSTA360) {
        camConnected = g_instaConnected;
    } else {
        camConnected = (pClient && pClient->isConnected());
    }

    CameraTelemetry t{};
    if (g_activeDriver) {
        portENTER_CRITICAL(&g_telemetryMux);
        t = g_activeDriver->getTelemetry();
        portEXIT_CRITICAL(&g_telemetryMux);
    }
    bool camReady = camConnected && (t.state == CAM_PAIRED || t.state == CAM_RECORDING);

    // 1. Arm Trigger (sampled on each MSP_STATUS packet, debounced over 2 consecutive reads)
    if (fcConnected) {
        uint32_t statusMs = g_bfBridge.getLastStatusMs();
        if (statusMs != 0 && statusMs != s_lastStatusSampleMs) {
            s_lastStatusSampleMs = statusMs;

            if (isArmed == s_lastCandidateArmed) {
                if (s_armCandidateCount < 255) s_armCandidateCount++;
            } else {
                s_lastCandidateArmed = isArmed;
                s_armCandidateCount = 1;
            }

            if (s_lastArmed != isArmed && (isArmed || s_armCandidateCount >= 2)) {
                s_lastArmed = isArmed;

                if (s_lastArmed) {
                    // Disarmed -> Armed edge
                    s_disarmTargetMs = 0; // Cancel pending stop immediately
                    char edgeBuf[80];
                    snprintf(edgeBuf, sizeof(edgeBuf), "ARM edge: autoRec=%d camReady=%d brand=%d hasMac=%d",
                             g_autoRecOnArm ? 1 : 0, camReady ? 1 : 0, (int)g_activeBrand,
                             (hasTargetAddress || hasSavedMacForActiveBrand) ? 1 : 0);
                    addLog("ARM", edgeBuf);
                    if (g_autoRecOnArm) {
                        if (camReady) {
                            if ((g_activeBrand == BRAND_INSTA360 && !s_armRecStarted) ||
                                (g_activeBrand != BRAND_INSTA360 && !t.isRecording)) {
                                enqueueCommand(CMD_RECORD_START);
                                s_armRecStarted = true;
                                addLog("ARM", "FC Armed -> Start Recording");
                            }
                        } else if (g_activeBrand == BRAND_DJI && (hasTargetAddress || hasSavedMacForActiveBrand)) {
                            g_pendingRecordOnWake = true;
                            g_pendingRecordExpireMs = now + 20000;
                            s_armRecStarted = true;
                            enqueueCommand(CMD_WAKE);
                            addLog("ARM", "Camera asleep -> WKP wake + auto-record");
                        }
                    }
                } else {
                    // Armed -> Disarmed edge
                    if (g_pendingRecordOnWake) {
                        g_pendingRecordOnWake = false;
                        s_armRecStarted = false;
                        s_disarmTargetMs = 0;
                    } else if (g_autoRecOnArm && s_armRecStarted) {
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
        if (camReady && ((g_activeBrand == BRAND_INSTA360 && s_armRecStarted) || (g_activeBrand != BRAND_INSTA360 && t.isRecording))) {
            enqueueCommand(CMD_RECORD_STOP);
            addLog("ARM", "Disarm delay expired -> Stop Recording");
        }
        s_armRecStarted = false;
    }

    // 3. AUX Channel Trigger (works even when disarmed for bench testing)
    if (fcConnected && g_auxRecChannel >= 1 && g_auxRecChannel <= 8) {
        uint16_t val = g_bfBridge.getAuxChannel(g_auxRecChannel);
        if (val >= 800) { // Ignore uninitialized channels (< 800)
            if (val >= 1700 && !s_auxHigh) {
                s_auxHigh = true;
                if (camReady) {
                    if ((g_activeBrand == BRAND_INSTA360 && !s_auxRecStarted) ||
                        (g_activeBrand != BRAND_INSTA360 && !t.isRecording)) {
                        enqueueCommand(CMD_RECORD_START);
                        s_auxRecStarted = true;
                        addLog("AUX", "AUX switch HIGH -> Start Recording");
                    }
                } else if (g_activeBrand == BRAND_DJI && (hasTargetAddress || hasSavedMacForActiveBrand)) {
                    g_pendingRecordOnWake = true;
                    g_pendingRecordExpireMs = now + 20000;
                    s_auxRecStarted = true;
                    enqueueCommand(CMD_WAKE);
                    addLog("AUX", "Camera asleep -> WKP wake + auto-record");
                }
            } else if (val <= 1300 && s_auxHigh) {
                s_auxHigh = false;
                if (g_pendingRecordOnWake) {
                    g_pendingRecordOnWake = false;
                    s_auxRecStarted = false;
                } else {
                    if (camReady) {
                        if ((g_activeBrand == BRAND_INSTA360 && s_auxRecStarted) ||
                            (g_activeBrand != BRAND_INSTA360 && t.isRecording)) {
                            enqueueCommand(CMD_RECORD_STOP);
                            addLog("AUX", "AUX switch LOW -> Stop Recording");
                        }
                    }
                    s_auxRecStarted = false;
                }
            }
        }
    }

    // 4. Pending Wake-Record Trigger
    if (g_pendingRecordOnWake) {
        if ((int32_t)(now - g_pendingRecordExpireMs) >= 0) {
            g_pendingRecordOnWake = false;
            s_armRecStarted = false;
            s_auxRecStarted = false;
        } else if (camReady) {
            g_pendingRecordOnWake = false;
            if (!t.isRecording) {
                enqueueCommand(CMD_RECORD_START);
                addLog("WAKE", "Camera awake & paired -> Starting recording");
            }
        }
    }
}

// ============================================================================
// Main Loop (Runs on loopTask)
// ============================================================================
void loop() {
    uint32_t now = millis();

    // 1. Update Status LED
    updateStatusLed(now);

    // 2. Scheduled Reboot Handler
    if (g_pendingRestart && (int32_t)(now - g_restartTargetMs) >= 0) {
        ESP.restart();
    }

    if (g_bootMode == MODE_WIFI_SETUP) {
        // ====================================================================
        // MODE 1: WI-FI SETUP LOOP
        // ====================================================================
        // Process Captive Portal DNS Requests
        for (int i = 0; i < 4; i++) {
            dnsServer.processNextRequest();
        }

        // WebSocket Client Cleanup
        static uint32_t lastWsCleanupMs = 0;
        if (now - lastWsCleanupMs >= 500) {
            lastWsCleanupMs = now;
            ws.cleanupClients();
        }

        // Drain and process Command Queue on loopTask
        if (g_cmdQueue) {
            CommandItem cmd;
            while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
                processCommand(cmd);
            }
        }

        // Flush pending logs to WebSocket clients
        flushLogsToWebSocket();

        // Rate-limited WebSocket Telemetry Broadcast (Max 5 Hz = every >= 200ms)
        static uint32_t lastWsStatePushMs = 0;
        if (now - lastWsStatePushMs >= 200) {
            if (hasStateChanged()) {
                broadcastStateJson();
                lastWsStatePushMs = now;
            }
        }

        g_bfBridge.loop();
        delay(2);
        return;
    }

    // ========================================================================
    // COMMON FOR BLE FLIGHT MODE & DEDICATED BLE BRIDGE MODE:
    // BOOT Button (GPIO 9) 2-Second Hold -> Switch to Wi-Fi Setup Mode
    // ========================================================================
    static uint32_t s_bootBtnLowStartMs = 0;
    if (digitalRead(9) == LOW) {
        if (s_bootBtnLowStartMs == 0) {
            s_bootBtnLowStartMs = now;
        } else if (now - s_bootBtnLowStartMs >= 2000) {
            s_bootBtnLowStartMs = 0;
            addLog("MODE", "BOOT button held 2s -> Switching to Wi-Fi Setup Mode");
            flushDiagHistoryToNvs();
            prefs.putUChar("boot_mode", (uint8_t)MODE_WIFI_SETUP);
            prefs.end();
            delay(50);
            ESP.restart();
        }
    } else {
        s_bootBtnLowStartMs = 0;
    }

    if (g_bootMode == MODE_BF_BLE_BRIDGE) {
        // ====================================================================
        // MODE 2: DEDICATED BETAFLIGHT BLE BRIDGE LOOP
        // ====================================================================
        if (g_cmdQueue) {
            CommandItem cmd;
            while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
                processCommand(cmd);
            }
        }

        g_bfBridge.loop();

        if (g_bfBleConnected) {
            if (!s_bfParamsUpdated && s_bfBleConnAtMs > 0 && (now - s_bfBleConnAtMs >= 600)) {
                s_bfParamsUpdated = true;
                if (pServer) {
                    pServer->updateConnParams(g_bfBleConnHandle, 6, 12, 0, 400);
                }
            }
            vTaskDelay(1);
        } else {
            delay(2);
        }
        return;
    }

    // ========================================================================
    // MODE 0: BLE FLIGHT MODE LOOP
    // ========================================================================

    // 1. ELRS-Style 60-Second Auto-Wi-Fi Timeout
    bool camConnected = (g_activeBrand == BRAND_INSTA360) ? g_instaConnected : (pClient && pClient->isConnected());
    if (!g_autoWifiCancelled && (camConnected || g_bfBridge.isArmed() || g_bfBleConnected)) {
        g_autoWifiCancelled = true;
        addLog("MODE", "Auto-Wi-Fi timeout cancelled (camera active, FC armed, or BF BLE connected)");
    }
    if (!g_autoWifiCancelled && (now - g_bootMs >= 60000UL)) {
        addLog("MODE", "No activity in 60s -> Switching to Wi-Fi Setup Mode");
        flushDiagHistoryToNvs();
        prefs.putUChar("boot_mode", (uint8_t)MODE_WIFI_SETUP);
        prefs.end();
        delay(50);
        ESP.restart();
    }

    // 2. Control NUS Advertising in Flight Mode for DJI/GoPro
    if (g_activeBrand != BRAND_INSTA360 && !isBroadcastingWake) {
        CameraTelemetry t{};
        if (g_activeDriver) {
            portENTER_CRITICAL(&g_telemetryMux);
            t = g_activeDriver->getTelemetry();
            portEXIT_CRITICAL(&g_telemetryMux);
        }
        bool camPaired = camConnected && (t.state == CAM_PAIRED || t.state == CAM_RECORDING);
        if (camPaired) {
            if (g_camPairedAtMs == 0) g_camPairedAtMs = now;
        } else {
            g_camPairedAtMs = 0;
        }

        bool idleStableForAdv = !camConnected && (g_lastDisconnectMs == 0 || (now - g_lastDisconnectMs >= 1000UL));
        bool shouldAdvNus = s_nusServerStarted && idleStableForAdv && !g_bfBridge.isArmed() && !g_bfBleConnected &&
                            !isBroadcastingWake && !g_pendingRecordOnWake && (connStep == CONN_IDLE) &&
                            (now - g_bootMs >= 4000UL);

        static uint32_t s_lastAdvTryMs = 0;
        static bool s_lastLoggedAdvState = false;
        NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
        if (pAdv) {
            if (shouldAdvNus && !pAdv->isAdvertising() && (now - s_lastAdvTryMs >= 1000UL)) {
                s_lastAdvTryMs = now;
                NimBLEScan* pScan = NimBLEDevice::getScan();
                bool wasScanning = pScan && pScan->isScanning();
                if (wasScanning) {
                    pScan->stop();
                    delay(20);
                }
                configureNusAdvertising();
                bool ok = pAdv->start();
                char diagEvt[32];
                snprintf(diagEvt, sizeof(diagEvt), "A@%lu ok=%d", (unsigned long)now, ok ? 1 : 0);
                appendDiagEvent(diagEvt, false);
                if (!s_lastLoggedAdvState) {
                    s_lastLoggedAdvState = true;
                    if (Serial && Serial.availableForWrite() >= 96) {
                        Serial.printf("[%lu][ADV] pAdv->start() -> ok=%d (isAdv=%d)\r\n",
                                      (unsigned long)now, ok ? 1 : 0, pAdv->isAdvertising() ? 1 : 0);
                    }
                }
                if (wasScanning) {
                    startBleScan();
                }
            } else if (!shouldAdvNus && pAdv->isAdvertising()) {
                pAdv->stop();
                if (s_lastLoggedAdvState) {
                    s_lastLoggedAdvState = false;
                    if (Serial && Serial.availableForWrite() >= 96) {
                        Serial.printf("[%lu][ADV] pAdv->stop() -> reason: shouldAdvNus=0 (camConn=%d step=%d armed=%d bfBle=%d)\r\n",
                                      (unsigned long)now, camConnected ? 1 : 0, (int)connStep, g_bfBridge.isArmed() ? 1 : 0, g_bfBleConnected ? 1 : 0);
                    }
                }
            }
        }

        static uint32_t s_lastStatPrintMs = 0;
        if (now - s_lastStatPrintMs >= 3000UL) {
            s_lastStatPrintMs = now;
            NimBLEScan* pScan = NimBLEDevice::getScan();
            int32_t bo = (int32_t)(s_cameraConnBackoffUntilMs - now);
            if (Serial && Serial.availableForWrite() >= 96) {
                Serial.printf("[%lu][STAT] camConn=%d step=%d scan=%d adv=%d shouldAdv=%d armed=%d bfBle=%d backoff=%ldms\r\n",
                              (unsigned long)now,
                              camConnected ? 1 : 0,
                              (int)connStep,
                              (pScan && pScan->isScanning()) ? 1 : 0,
                              (pAdv && pAdv->isAdvertising()) ? 1 : 0,
                              shouldAdvNus ? 1 : 0,
                              g_bfBridge.isArmed() ? 1 : 0,
                              g_bfBleConnected ? 1 : 0,
                              (long)(bo > 0 ? bo : 0));
            }
        }
    }

    // 3. Deferred BLE Connect Processing for Insta360
    if (g_instaConnectPending) {
        g_instaConnectPending = false;
        if (g_instaConnected) {
            g_instaConnHandle = g_stagedInstaConnHandle;
            s_pendingInstaAddr = g_stagedInstaAddr;
            s_instaMacSaveAtMs = now + 500;
            g_activeDriver->onConnected(g_instaConnHandle, bleWriteCallback);
            portENTER_CRITICAL(&g_telemetryMux);
            g_telemetryVersion++;
            portEXIT_CRITICAL(&g_telemetryMux);
        }
    }

    // 4. Deferred BLE Disconnect Processing
    if (g_disconnectPending) {
        g_disconnectPending = false;
        s_hadDisconnectThisBoot = true;
        s_macSavedThisSession = false;
        s_diagPairedLoggedThisSession = false;
        g_camPairedAtMs = 0;
        g_lastDisconnectMs = millis();
        bool wasShutdown = s_expectingShutdownDisconnect;
        s_expectingShutdownDisconnect = false;
        s_cameraConnBackoffUntilMs = now + (wasShutdown ? 2200UL : 200UL);
        s_instaMacSaveAtMs = 0;
        g_explicitSleepRequested = false;

        g_cameraAsleep = false;

        recordFirstDisconnect("BLE_EVENT", g_lastDisconnectReason, now);
        char discMsg[64];
        snprintf(discMsg, sizeof(discMsg), "Link disconnected (reason=0x%02X); active search resumed", (unsigned)g_lastDisconnectReason);
        addLog("BLE", discMsg);
        connStep = CONN_IDLE;
        pWriteChar = nullptr;
        pNotifyChar = nullptr;
        pGoproSettingsWriteChar = nullptr;
        pGoproSettingsNotifyChar = nullptr;
        pGoproQueryWriteChar = nullptr;
        pGoproQueryNotifyChar = nullptr;
        s_armRecStarted = false;
        s_auxRecStarted = false;
        g_activeDriver->onDisconnected();
        g_telemetryVersion++;

        if (g_activeBrand != BRAND_INSTA360) {
            if (pClient) {
                NimBLEDevice::deleteClient(pClient);
                pClient = nullptr;
            }
            g_bleScanAllowedAfterMs = now + 100;
            lastScanRestartMs = now;
        } else {
            NimBLEDevice::getAdvertising()->start();
        }
    }

    // 5. Drain and process Command Queue on loopTask
    if (g_cmdQueue) {
        CommandItem cmd;
        while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
            processCommand(cmd);
        }
    }

    // 6. Handle WKP Wake Broadcast Timeout / Early Camera Detection
    if (isBroadcastingWake) {
        handleWkpWakeLoop(millis());
        if (isBroadcastingWake) {
            g_bfBridge.loop();
            runArmAndAuxController(millis());
            delay(2);
            return;
        }
        now = millis();
    }

    // 7. Run Non-Blocking Connection State Machine (Central mode)
    if (g_activeBrand != BRAND_INSTA360 && connStep != CONN_IDLE) {
        runConnectionStateMachine(now);
    }

    // 8. Periodic Driver Updates (Telemetry watchdogs, heartbeats)
    if (g_activeDriver) {
        g_activeDriver->updateLoop(now);

        // Check if driver requested link teardown (camera sleep / shutdown)
        if (g_activeDriver->shouldDisconnect()) {
            if (pClient && pClient->isConnected()) {
                g_explicitSleepRequested = false;
                g_cameraAsleep = false;
                s_expectingShutdownDisconnect = true;
                g_lastDisconnectMs = now;
                addLog("BLE", "Camera shutdown / sleep confirmed -> Closing link");
                pClient->disconnect();
            }
        }

        // Telemetry silence watchdog:
        // - 2.5s in CAM_CONNECTING (3 handshake retries failed)
        // - 3.5s in CAM_PAIRED/CAM_RECORDING (safely past DjiDriver's 2.0s retry window)
        if (g_activeBrand == BRAND_DJI && pClient && pClient->isConnected()) {
            CameraTelemetry curT{};
            portENTER_CRITICAL(&g_telemetryMux);
            curT = g_activeDriver->getTelemetry();
            portEXIT_CRITICAL(&g_telemetryMux);
            uint32_t lastTelem = g_activeDriver->getLastTelemetryMs();
            if (lastTelem > 0) {
                int32_t silenceMs = (int32_t)(millis() - lastTelem);
                if ((curT.state == CAM_CONNECTING && silenceMs >= 2500) ||
                    ((curT.state == CAM_PAIRED || curT.state == CAM_RECORDING) && silenceMs >= 3500)) {
                    g_lastDisconnectMs = millis();
                    addLog("BLE", "Telemetry timeout -> Dropping dead link");
                    pClient->disconnect();
                }
            }
        }
    }

    // Deferred MAC saving once camera is confirmed paired/authenticated
    if (g_activeBrand != BRAND_INSTA360 && camConnected) {
        CameraTelemetry t{};
        if (g_activeDriver) {
            portENTER_CRITICAL(&g_telemetryMux);
            t = g_activeDriver->getTelemetry();
            portEXIT_CRITICAL(&g_telemetryMux);
        }
        if (t.state == CAM_PAIRED || t.state == CAM_RECORDING) {
            g_cameraAsleep = false;
            if (!s_diagPairedLoggedThisSession) {
                s_diagPairedLoggedThisSession = true;
                addLog("BLE", "Camera paired & ready");
                char diagEvt[64];
                snprintf(diagEvt, sizeof(diagEvt), "P@%lu", (unsigned long)now);
                appendDiagEvent(diagEvt, false);
            }
            if (!s_macSavedThisSession) {
                s_macSavedThisSession = true;
                saveActiveBrandMac(targetAddress);
            }
        }
    }

    // Deferred Insta360 MAC saving
    if (g_instaConnected && s_instaMacSaveAtMs > 0 && (int32_t)(now - s_instaMacSaveAtMs) >= 0) {
        s_instaMacSaveAtMs = 0;
        saveActiveBrandMac(s_pendingInstaAddr);
    }

    // 9. Time-sliced BLE scanning (Central mode only, paused while Betaflight BLE is connected)
    if (g_bfBleConnected) {
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan && pScan->isScanning()) {
            pScan->stop();
        }
    } else if (g_activeBrand != BRAND_INSTA360) {
        if ((!pClient || !pClient->isConnected()) && connStep == CONN_IDLE && !isBroadcastingWake) {
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (!pScan->isScanning() && (now - lastScanRestartMs >= 15)) {
                startBleScan();
            }
        }
    }

    // 10. Run Betaflight Bridge (UART MSP monitor & BLE forwarder)
    g_bfBridge.loop();

    // 11. Edge-Triggered Recording Controller
    runArmAndAuxController(now);

    if (g_bfBleConnected) {
        if (!s_bfParamsUpdated && s_bfBleConnAtMs > 0 && (now - s_bfBleConnAtMs >= 600)) {
            s_bfParamsUpdated = true;
            if (pServer) {
                pServer->updateConnParams(g_bfBleConnHandle, 6, 12, 0, 400);
            }
        }
        vTaskDelay(1);
    } else if (connStep != CONN_IDLE) {
        vTaskDelay(1);
    } else {
        delay(2);
    }
}
