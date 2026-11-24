#include "DjiDriver.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cstdio>
#include <cstring>

// ============================================================================
// Official DJI R-SDK Protocol Structures (Osmo-GPS-Controller-Demo Spec)
// ============================================================================
static constexpr uint32_t CONTROLLER_DEVICE_ID = 0x00123456;

struct __attribute__((packed)) DjiConnectionRequest {
    uint32_t device_id;    // 0x00123456
    uint8_t  mac_addr_len; // 6
    uint8_t  mac_addr[16]; // Controller MAC (MSB-first)
    uint32_t fw_version;   // 0
    uint8_t  conidx;       // 0
    uint8_t  verify_mode;  // 1 = Pair request dialog
    uint16_t verify_data;  // 1234
    uint8_t  reserved[4];  // 0
};

struct __attribute__((packed)) DjiConnectionResponse {
    uint32_t device_id;    // 0x00123456
    uint8_t  ret_code;     // 0 = OK
    uint8_t  reserved[4];  // 0
};

struct __attribute__((packed)) DjiRecordControl {
    uint32_t device_id;    // 0x00123456
    uint8_t  record_ctrl;  // 0 = Start, 1 = Stop
    uint8_t  reserved[4];  // 0
};

struct __attribute__((packed)) DjiStatusSubscription {
    uint8_t push_mode;     // 3 = Periodic + on change, 2 = Periodic, 1 = Single, 0 = Off
    uint8_t push_freq;     // 20 = 2.0 Hz
    uint8_t reserved[4];   // 0
};

struct __attribute__((packed)) DjiModeSwitch {
    uint32_t device_id;    // 0x00123456
    uint8_t  mode;         // 0x00=SlowMo, 0x01=Video, 0x02=Timelapse, 0x05=Photo, 0x28=LowLight
    uint8_t  reserved[4];  // 0
};

struct __attribute__((packed)) DjiKeyReport {
    uint8_t  key_code;     // 0x01 = Shutter, 0x02 = QS, 0x03 = Snapshot
    uint8_t  mode;         // 0x01 = Key Event
    uint16_t key_value;    // 0x0000 = Short press, 0x0001 = Long press
};

// Exact packed layout of CmdSet=0x00, CmdId=0x17 (GPS Data per Osmo-GPS-Controller-Demo)
struct __attribute__((packed)) DjiGpsData {
    int32_t  date;               // year * 10000 + month * 100 + day
    int32_t  time;               // (hour + 8) * 10000 + minute * 100 + second
    int32_t  longitude;          // actual longitude * 1e7
    int32_t  latitude;           // actual latitude * 1e7
    int32_t  height;             // elevation / height in millimeters (mm)
    float    speed_north;        // velocity vector North in cm/s
    float    speed_east;         // velocity vector East in cm/s
    float    speed_down;         // velocity vector Down in cm/s
};

// Exact packed binary layout of CmdSet=0x1D, CmdId=0x02 (camera_status_push)
struct __attribute__((packed)) DjiStatusPushPayload {
    uint8_t  camera_mode;           // 0: 0x00=SlowMo, 0x01=Video, 0x02=Timelapse, 0x05=Photo, 0x0A=Hyperlapse, 0x28=Low light
    uint8_t  camera_status;         // 1: 0x00=Screen off, 0x01=Live view, 0x02=Playback, 0x03=Recording/Photo in progress, 0x05=Pre-recording
    uint8_t  video_resolution;      // 2: 10=1080P, 16=4K 16:9, 45=2.7K 16:9, 66=1080P 9:16, 67=2.7K 9:16, 95=2.7K 4:3, 103=4K 4:3, 109=4K 9:16
    uint8_t  fps_idx;               // 3: 1=24fps, 2=25fps, 3=30fps, 4=48fps, 5=50fps, 6=60fps, 10=100fps, 7=120fps, 19=200fps, 8=240fps
    uint8_t  eis_mode;              // 4: 0=Off, 1=RS, 2=HS, 3=RS+, 4=HB
    uint16_t record_time;          // 5..6: elapsed recording time in seconds
    uint8_t  fov_type;              // 7
    uint8_t  photo_ratio;           // 8: 0=4:3, 1=16:9
    uint16_t real_time_countdown;  // 9..10: seconds
    uint16_t timelapse_interval;   // 11..12: in 0.1s
    uint16_t timelapse_duration;   // 13..14: in seconds
    uint32_t remain_capacity;      // 15..18: SD card remaining in MB
    uint32_t remain_photo_num;     // 19..22: remaining photos count
    uint32_t remain_time;          // 23..26: remaining record seconds
    uint8_t  user_mode;             // 27: 0=General, 1..5=Custom 1..5
    uint8_t  power_mode;            // 28: 0=Normal, 3=Sleep
    uint8_t  camera_mode_next_flag; // 29
    uint8_t  temp_over;             // 30: 0=Normal, 1=Warning, 2=High, 3=Overheat
    uint32_t photo_countdown_ms;   // 31..34: ms
    uint16_t loop_record_sends;    // 35..36: seconds
    uint8_t  camera_bat_percentage; // 37: Battery percentage (0..100%)
};

// ============================================================================
// Constructor & Destructor
// ============================================================================
DjiDriver::DjiDriver()
    : m_writeFn(nullptr)
    , m_connHandle(0)
    , m_seq(1)
    , m_hasSavedMac(false)
    , m_lastTelemetryMs(0)
    , m_lastRetryMs(0)
    , m_wakeUntilMs(0)
    , m_isWaking(false)
    , m_pendingHandshake(false)
    , m_connectedMs(0)
    , m_pendingCameraConnAck(false)
    , m_pendingCameraConnSeq(0)
    , m_postAuthInitNeeded(false)
    , m_postAuthMs(0)
    , m_needsDisconnect(false)
    , m_sleepRequestedMs(0)
    , m_sleepPending(false)
    , m_lastPowerMode(0)
    , m_lastCamStatus(0)
{
    memset(&m_telemetry, 0, sizeof(m_telemetry));
    m_telemetry.state = CAM_DISCONNECTED;
    strcpy(m_telemetry.currentMode, "NORMAL VIDEO");
    strcpy(m_telemetry.resolutionFps, "4K / 60 FPS");
    strcpy(m_telemetry.stabilization, "ROCKSTEADY");
    strcpy(m_telemetry.tempStatus, "NORMAL");
    strcpy(m_telemetry.modelName, "DJI Osmo Action");
    strcpy(m_telemetry.lastHexPacket, "--");

    m_caps.supportsWkpWake = true;
    m_caps.supportsHilight = true;
    m_caps.supportsKeyReport = true;
    m_caps.supportsTelemetryPush = true;
    m_caps.supportsModeSwitch = true;
    m_caps.modeCount = 6;
    m_caps.availableModes[0] = "NORMAL VIDEO";
    m_caps.availableModes[1] = "LOW LIGHT (NIGHT)";
    m_caps.availableModes[2] = "SLOW MOTION";
    m_caps.availableModes[3] = "TIMELAPSE";
    m_caps.availableModes[4] = "PHOTO";
    m_caps.availableModes[5] = "HYPERLAPSE";
}

DjiDriver::~DjiDriver() {}

void DjiDriver::setSavedMac(const uint8_t* mac) {
    if (mac) {
        memcpy(m_savedMac, mac, 6);
        m_hasSavedMac = true;
        snprintf(m_telemetry.cameraMac, sizeof(m_telemetry.cameraMac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

// ============================================================================
// Checksum Algorithms
// ============================================================================
uint16_t DjiDriver::calculateCrc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x3AA3;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t DjiDriver::calculateCrc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0x00003AA3UL;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Advertisement Matcher
// ============================================================================
bool DjiDriver::isMatchingAdvertisement(const char* name, const uint8_t* mfgData, size_t mfgLen, const uint8_t* serviceUuid16) {
    if (name && (strstr(name, "OsmoAction") || strstr(name, "Action") || strstr(name, "DJI") || strstr(name, "Pocket"))) {
        return true;
    }
    if (serviceUuid16 && serviceUuid16[0] == 0xF0 && serviceUuid16[1] == 0xFF) {
        return true; // 0xFFF0
    }
    if (mfgData && mfgLen >= 2 && mfgData[0] == 0xAA && mfgData[1] == 0x08) {
        return true; // DJI Manufacturer ID 0x08AA
    }
    return false;
}

void DjiDriver::configureSecurity() {
    // DJI Action requires OPEN GATT (no BLE SMP bonding)
}

// ============================================================================
// Frame Assembly & Transmission
// ============================================================================
void DjiDriver::sendPacketWithSeq(uint8_t cmdType, uint8_t cmdSet, uint8_t cmdId, uint16_t seq, const uint8_t* payload, size_t payloadLen) {
    if (!m_writeFn) return;

    // Full R-SDK Frame:
    // [0xAA:1][verLen:2][cmdType:1][enc:1][res:3][seq:2][crc16:2][cmdSet:1][cmdId:1][payload:N][crc32:4]
    // Total = 10 (header) + 2 (crc16) + 2 (cmd) + payloadLen + 4 (crc32) = 18 + payloadLen
    const size_t totalLen = 14 + payloadLen + 4;
    if (totalLen > 240) return;

    uint8_t buf[240];
    memset(buf, 0, totalLen);

    buf[0] = 0xAA;
    uint16_t verLen = (uint16_t)(totalLen & 0x03FF);
    buf[1] = (uint8_t)(verLen & 0xFF);
    buf[2] = (uint8_t)((verLen >> 8) & 0xFF);
    buf[3] = cmdType;
    buf[4] = 0x00; // enc = none
    // buf[5..7] = 0
    buf[8] = (uint8_t)(seq & 0xFF);
    buf[9] = (uint8_t)((seq >> 8) & 0xFF);

    uint16_t crc16 = calculateCrc16(buf, 10);
    buf[10] = (uint8_t)(crc16 & 0xFF);
    buf[11] = (uint8_t)((crc16 >> 8) & 0xFF);

    buf[12] = cmdSet;
    buf[13] = cmdId;
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(&buf[14], payload, payloadLen);
    }

    uint32_t crc32 = calculateCrc32(buf, 14 + payloadLen);
    size_t crcOff = 14 + payloadLen;
    buf[crcOff + 0] = (uint8_t)(crc32 & 0xFF);
    buf[crcOff + 1] = (uint8_t)((crc32 >> 8) & 0xFF);
    buf[crcOff + 2] = (uint8_t)((crc32 >> 16) & 0xFF);
    buf[crcOff + 3] = (uint8_t)((crc32 >> 24) & 0xFF);

    // Format hex for UI Inspector
    char hex[96];
    size_t hlen = 0;
    for (size_t i = 0; i < totalLen && i < 16; i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", buf[i]);
    }
    if (totalLen > 16) snprintf(hex + hlen, sizeof(hex) - hlen, "...");
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);
    m_telemetry.packetCount++;

    // Write to FFF5 characteristic (without response per DJI R-SDK spec)
    m_writeFn(0xFFF5, buf, totalLen, false);
}

void DjiDriver::sendPacket(uint8_t cmdType, uint8_t cmdSet, uint8_t cmdId, const uint8_t* payload, size_t payloadLen) {
    sendPacketWithSeq(cmdType, cmdSet, cmdId, m_seq++, payload, payloadLen);
}

// ============================================================================
// Connection Lifecycle & Handshake
// ============================================================================
bool DjiDriver::onConnected(uint16_t connHandle, BleWriteFunc writeFn) {
    m_connHandle = connHandle;
    m_writeFn = writeFn;
    m_telemetry.state = CAM_CONNECTING;
    m_telemetry.batteryPercent = 0;
    m_telemetry.packetCount = 0;
    m_lastPowerMode = 0;
    m_lastCamStatus = 0;
    m_seq = 1;
    m_lastTelemetryMs = millis();
    m_lastRetryMs = millis();
    m_pendingHandshake = false;
    m_pendingCameraConnAck = false;
    m_postAuthInitNeeded = false;
    m_needsDisconnect = false;

    // Send R-SDK pairing handshake immediately on connection (verify_mode = 0: fast history link)
    sendPairingHandshake(0);
    return true;
}

void DjiDriver::onDisconnected() {
    m_connHandle = 0;
    m_writeFn = nullptr;
    m_telemetry.state = CAM_DISCONNECTED;
    m_telemetry.isRecording = false;
    m_pendingHandshake = false;
    m_pendingCameraConnAck = false;
    m_postAuthInitNeeded = false;
    m_needsDisconnect = false;
    m_sleepPending = false;
    m_sleepRequestedMs = 0;
    m_lastRetryMs = 0;
    m_lastPowerMode = 0;
    m_lastCamStatus = 0;
}

void DjiDriver::sendPairingHandshake(uint8_t verifyMode) {
    DjiConnectionRequest req{};
    req.device_id    = CONTROLLER_DEVICE_ID;
    req.mac_addr_len = 6;

    // Put ESP32-C3's own BLE MAC into req.mac_addr (MSB-first)
    NimBLEAddress ownAddr = NimBLEDevice::getAddress();
    const uint8_t* rawMac = ownAddr.getVal();
    for (int i = 0; i < 6; i++) {
        req.mac_addr[i] = rawMac[5 - i];
    }

    req.fw_version  = 0;
    req.conidx      = 0;
    req.verify_mode = verifyMode; // 0 = reconnect/history check (auto prompts if unbonded), 1 = force pair dialog
    req.verify_data = 1234;       // Verification PIN code

    // CmdType = 0x02 (Wait ACK), CmdSet = 0x00, CmdId = 0x19
    sendPacket(0x02, 0x00, 0x19, (const uint8_t*)&req, sizeof(req));
}

// ============================================================================
// Camera Commands
// ============================================================================
void DjiDriver::startRecording() {
    DjiRecordControl req{};
    req.device_id   = CONTROLLER_DEVICE_ID;
    req.record_ctrl = 0; // 0 = Start recording / shutter
    sendPacket(0x02, 0x1D, 0x03, (const uint8_t*)&req, sizeof(req));
}

void DjiDriver::stopRecording() {
    DjiRecordControl req{};
    req.device_id   = CONTROLLER_DEVICE_ID;
    req.record_ctrl = 1; // 1 = Stop recording
    sendPacket(0x02, 0x1D, 0x03, (const uint8_t*)&req, sizeof(req));
}

void DjiDriver::toggleShutter() {
    if (m_telemetry.isRecording) {
        stopRecording();
    } else {
        startRecording();
    }
}

void DjiDriver::sendKeyEvent() {
    sendKeyReport(0x01, 0x0000); // Shutter key short press
}

void DjiDriver::sendKeyReport(uint8_t keyCode, uint16_t keyValue) {
    DjiKeyReport req{};
    req.key_code  = keyCode;   // 0x01 = Shutter, 0x02 = QS, 0x03 = Snapshot
    req.mode      = 0x01;      // Key event
    req.key_value = keyValue;  // 0x0000 = Short press, 0x0001 = Long press
    sendPacket(0x02, 0x00, 0x11, (const uint8_t*)&req, sizeof(req));
}

void DjiDriver::tagHilight() {
    sendKeyReport(0x02, 0x0000); // QS short press highlights while recording
}

void DjiDriver::setMode(uint8_t modeIndex) {
    // Mode mappings from official SDK:
    // 0 -> 0x01 (Normal Video)
    // 1 -> 0x28 (Low light / Super Night / HDR)
    // 2 -> 0x00 (Slow Motion)
    // 3 -> 0x02 (Timelapse)
    // 4 -> 0x05 (Photo)
    // 5 -> 0x0A (Hyperlapse)
    uint8_t modeVal = 0x01;
    if (modeIndex == 1) modeVal = 0x28;
    else if (modeIndex == 2) modeVal = 0x00;
    else if (modeIndex == 3) modeVal = 0x02;
    else if (modeIndex == 4) modeVal = 0x05;
    else if (modeIndex == 5) modeVal = 0x0A;

    DjiModeSwitch req{};
    req.device_id = CONTROLLER_DEVICE_ID;
    req.mode      = modeVal;
    sendPacket(0x02, 0x1D, 0x04, (const uint8_t*)&req, sizeof(req));
}

void DjiDriver::setStabilization(uint8_t eisMode) {
    // DJI Action does not support EIS writes over BLE; EIS is read-only from 1D/02 telemetry push.
    (void)eisMode;
}

void DjiDriver::powerSleep() {
    uint8_t payload[] = {0x03}; // 0x03 = Sleep (per DJI R-SDK spec)
    sendPacket(0x02, 0x00, 0x1A, payload, sizeof(payload));
    m_telemetry.state = CAM_SLEEPING;
    m_telemetry.isRecording = false;
    m_sleepRequestedMs = millis();
    m_sleepPending = true;
}

void DjiDriver::powerNormal() {
    uint8_t payload[] = {0x00}; // 0x00 = Normal (wake up / power normal per DJI R-SDK spec)
    sendPacket(0x02, 0x00, 0x1A, payload, sizeof(payload));
    m_telemetry.state = CAM_PAIRED;
}

void DjiDriver::sendGpsData(int32_t date, int32_t time, int32_t lon, int32_t lat, int32_t heightMm, float speedN, float speedE, float speedD) {
    DjiGpsData gps{};
    gps.date = date;
    gps.time = time;
    gps.longitude = lon;
    gps.latitude = lat;
    gps.height = heightMm;
    gps.speed_north = speedN;
    gps.speed_east = speedE;
    gps.speed_down = speedD;
    // CmdSet=0x00, CmdId=0x17 (NO_ACK per continuous GPS push)
    sendPacket(0x00, 0x00, 0x17, (const uint8_t*)&gps, sizeof(gps));
}

void DjiDriver::sendMockGps() {
    // Bench mock coordinates: San Francisco 37.7749 N, -122.4194 W, 15m alt, 3.5 m/s east
    sendGpsData(20261120, 200000, -1224194000, 377749000, 15000, 0.0f, 350.0f, 0.0f);
}

void DjiDriver::wakeCamera() {
    m_isWaking = true;
    m_wakeUntilMs = 2000;
}

void DjiDriver::setPushMode(uint8_t pushMode) {
    DjiStatusSubscription req{};
    req.push_mode = pushMode; // 3 = Periodic 2Hz + status change
    req.push_freq = 20;       // 2.0 Hz
    sendPacket(0x00, 0x1D, 0x05, (const uint8_t*)&req, sizeof(req)); // 0x00 = NO_ACK per DJI R-SDK spec
}

void DjiDriver::sendRawHex(const uint8_t* bytes, size_t len) {
    if (m_writeFn && bytes && len > 0) {
        m_writeFn(0xFFF5, bytes, len, false);
    }
}

// ============================================================================
// Incoming Notification Handler (0xFFF4)
// ============================================================================
void DjiDriver::onGattNotify(uint16_t handle, const uint8_t* data, size_t len) {
    if (len < 18 || data[0] != 0xAA) return;

    m_telemetry.packetCount++;
    m_lastTelemetryMs = millis(); // Refresh watchdog with current wall clock

    uint8_t  cmdType = data[3];
    uint16_t seq     = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
    uint8_t  cmdSet  = data[12];
    uint8_t  cmdId   = data[13];
    const uint8_t* payload = &data[14];
    size_t payloadLen = (len >= 18) ? (len - 18) : 0;

    // CmdSet=0x00, CmdId=0x19: Connection Request / Response
    if (cmdSet == 0x00 && cmdId == 0x19) {
        bool isResponse = (cmdType & 0x20) != 0;
        if (isResponse) {
            // Camera ACK'd our Connection Request (Step 2 of handshake)
            // Camera will now follow up with Step 3 (counter-request with verify_mode=2)
        } else if (payloadLen >= sizeof(DjiConnectionRequest)) {
            // Camera sent counter-request with verify_mode=2 (Step 3 of handshake)
            DjiConnectionRequest camReq{};
            memcpy(&camReq, payload, sizeof(camReq));

            // Queue Step 4 ACK to be sent safely from updateLoop() outside interrupt callback
            m_pendingCameraConnSeq = seq;
            m_pendingCameraConnAck = true;

            if (camReq.verify_data == 0) { // 0 = ALLOWED
                m_postAuthInitNeeded = true;
                m_postAuthMs = millis();
            }
        }
    }
    // CmdSet=0x1D, CmdId=0x02: Camera Status Push (Telemetry)
    else if (cmdSet == 0x1D && cmdId == 0x02) {
        m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
        parseTelemetryPayload(payload, payloadLen);
    }
    // CmdSet=0x1D, CmdId=0x06: New Camera Status Push (ASCII strings)
    else if (cmdSet == 0x1D && cmdId == 0x06) {
        m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
        if (payloadLen >= 3 && payload[0] == 0x01) {
            uint8_t nameLen = payload[1];
            if (nameLen > 0 && nameLen <= 20 && payloadLen >= 2 + nameLen) {
                char modeName[24] = {0};
                memcpy(modeName, &payload[2], nameLen);
                strncpy(m_telemetry.currentMode, modeName, sizeof(m_telemetry.currentMode) - 1);
            }
        }
    }
    // CmdSet=0x1D, CmdId=0x03: Record Control Response ACK
    else if (cmdSet == 0x1D && cmdId == 0x03) {
        m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
    }
}

// ============================================================================
// Telemetry Parsing (Byte-Exact to DJI R-SDK Struct)
// ============================================================================
void DjiDriver::parseTelemetryPayload(const uint8_t* payload, size_t len) {
    if (len >= sizeof(DjiStatusPushPayload)) {
        const DjiStatusPushPayload* p = reinterpret_cast<const DjiStatusPushPayload*>(payload);

        // Battery Percentage (Byte 37)
        if (p->camera_bat_percentage <= 100) {
            m_telemetry.batteryPercent = p->camera_bat_percentage;
        }

        m_lastPowerMode = p->power_mode;
        m_lastCamStatus = p->camera_status;

        // Recording Status (Byte 1: 0x03 = Shooting / Recording in progress)
        m_telemetry.isRecording = (p->camera_status == 0x03);
        if (p->power_mode == 3) {
            m_telemetry.state = CAM_SLEEPING;
            m_telemetry.isRecording = false;
            m_needsDisconnect = true;
        } else {
            m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
        }

        // Duration & Remaining Metrics
        m_telemetry.recordTimeSec    = p->record_time;
        m_telemetry.remainCapacityMB = p->remain_capacity;
        m_telemetry.remainPhotos     = p->remain_photo_num;
        m_telemetry.remainVideoSec   = p->remain_time;

        // Camera Mode (Byte 0)
        switch (p->camera_mode) {
            case 0x00: strcpy(m_telemetry.currentMode, "SLOW MOTION"); break;
            case 0x01: strcpy(m_telemetry.currentMode, "NORMAL VIDEO"); break;
            case 0x02: strcpy(m_telemetry.currentMode, "TIMELAPSE"); break;
            case 0x05: strcpy(m_telemetry.currentMode, "PHOTO"); break;
            case 0x0A: strcpy(m_telemetry.currentMode, "HYPERLAPSE"); break;
            case 0x28: strcpy(m_telemetry.currentMode, "LOW LIGHT (NIGHT)"); break;
            default: snprintf(m_telemetry.currentMode, sizeof(m_telemetry.currentMode), "MODE 0x%02X", p->camera_mode); break;
        }

        // Electronic Image Stabilization (Byte 4)
        switch (p->eis_mode) {
            case 0: strcpy(m_telemetry.stabilization, "OFF"); break;
            case 1: strcpy(m_telemetry.stabilization, "ROCKSTEADY"); break;
            case 2: strcpy(m_telemetry.stabilization, "HORIZONSTEADY"); break;
            case 3: strcpy(m_telemetry.stabilization, "ROCKSTEADY+"); break;
            case 4: strcpy(m_telemetry.stabilization, "HORIZONBALANCING"); break;
            default: strcpy(m_telemetry.stabilization, "EIS ACTIVE"); break;
        }

        // Temperature Warning Status (Byte 30)
        switch (p->temp_over) {
            case 0: strcpy(m_telemetry.tempStatus, "NORMAL"); m_telemetry.temperatureC = 38; break;
            case 1: strcpy(m_telemetry.tempStatus, "WARM (WARNING)"); m_telemetry.temperatureC = 55; break;
            case 2: strcpy(m_telemetry.tempStatus, "HIGH (RECORD PAUSED)"); m_telemetry.temperatureC = 68; break;
            case 3: strcpy(m_telemetry.tempStatus, "OVERHEAT (SHUTDOWN)"); m_telemetry.temperatureC = 80; break;
            default: strcpy(m_telemetry.tempStatus, "NORMAL"); m_telemetry.temperatureC = 38; break;
        }

        // Resolution & Frame Rate (Bytes 2 & 3)
        const char* resStr = "HD";
        if (p->video_resolution == 16 || p->video_resolution == 103 || p->video_resolution == 109) resStr = "4K";
        else if (p->video_resolution == 45 || p->video_resolution == 67 || p->video_resolution == 95) resStr = "2.7K";
        else if (p->video_resolution == 10 || p->video_resolution == 66) resStr = "1080P";

        const char* fpsStr = "60";
        switch (p->fps_idx) {
            case 1: fpsStr = "24"; break;
            case 2: fpsStr = "25"; break;
            case 3: fpsStr = "30"; break;
            case 4: fpsStr = "48"; break;
            case 5: fpsStr = "50"; break;
            case 6: fpsStr = "60"; break;
            case 7: fpsStr = "120"; break;
            case 8: fpsStr = "240"; break;
            case 10: fpsStr = "100"; break;
            case 19: fpsStr = "200"; break;
        }
        snprintf(m_telemetry.resolutionFps, sizeof(m_telemetry.resolutionFps), "%s / %s FPS", resStr, fpsStr);
    } else if (len >= 6) {
        // Fallback for partial packets
        uint8_t status = payload[1];
        m_lastCamStatus = status;
        m_telemetry.isRecording = (status == 0x03);
        m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
        if (len >= 7) {
            m_telemetry.recordTimeSec = (uint32_t)payload[5] | ((uint32_t)payload[6] << 8);
        }
    }
}

// ============================================================================
// Watchdog, Handshake State Machine & Keep-Alive
// ============================================================================
void DjiDriver::updateLoop(uint32_t nowMs) {
    // 0. Delay 300ms after sleep command before tearing down link
    if (m_sleepPending && (nowMs - m_sleepRequestedMs >= 300)) {
        m_sleepPending = false;
        m_needsDisconnect = true;
    }

    // Handshake Retry while connecting
    if (m_telemetry.state == CAM_CONNECTING) {
        if (nowMs - m_lastTelemetryMs >= 800 && nowMs - m_lastRetryMs >= 800) {
            m_lastRetryMs = nowMs;
            sendPairingHandshake(0);
        }
    }

    // 1. Reply to Camera Counter-Request (Step 4 of Handshake)
    if (m_pendingCameraConnAck) {
        m_pendingCameraConnAck = false;
        DjiConnectionResponse resp{};
        resp.device_id = CONTROLLER_DEVICE_ID;
        resp.ret_code  = 0x00; // OK
        // Send ACK echoing camera's exact sequence number
        sendPacketWithSeq(0x20, 0x00, 0x19, m_pendingCameraConnSeq, (const uint8_t*)&resp, sizeof(resp));
    }

    // 2. Post-Auth Initialization: Subscribe to 2Hz live telemetry push immediately
    if (m_postAuthInitNeeded && (nowMs - m_postAuthMs >= 300)) {
        m_postAuthInitNeeded = false;
        m_telemetry.state = CAM_PAIRED;
        m_lastRetryMs = nowMs;
        setPushMode(0x03); // Subscribe to 2 Hz live telemetry push
    }

    // 3. Keep-Alive & Telemetry Retry Watchdog:
    // Once paired, retry setPushMode until live telemetry arrives (batteryPercent > 0),
    // then retry if telemetry pauses for > 2 seconds.
    if (m_telemetry.state >= CAM_PAIRED) {
        if (m_telemetry.batteryPercent == 0) {
            if (nowMs - m_lastRetryMs >= 1000) {
                m_lastRetryMs = nowMs;
                setPushMode(0x03);
            }
        } else if (m_lastTelemetryMs > 0 && (nowMs - m_lastTelemetryMs > 2000) && (nowMs - m_lastRetryMs >= 2000)) {
            m_lastRetryMs = nowMs;
            setPushMode(0x03);
        }
    }
}
