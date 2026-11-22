#include "GoproDriver.h"
#include <cstdio>
#include <cstring>
#include <Arduino.h>

GoproDriver::GoproDriver()
    : m_writeFn(nullptr)
    , m_connHandle(0)
    , m_hasSavedMac(false)
    , m_lastQueryMs(0)
    , m_lastKeepAliveMs(0)
    , m_pendingSettingsPush(false)
    , m_connectedMs(0)
    , m_rxLen(0)
    , m_rxExpected(0)
{
    memset(&m_telemetry, 0, sizeof(m_telemetry));
    m_telemetry.state = CAM_DISCONNECTED;
    strcpy(m_telemetry.currentMode, "STANDARD VIDEO");
    strcpy(m_telemetry.resolutionFps, "4K / 60 FPS");
    strcpy(m_telemetry.stabilization, "HYPERSMOOTH");
    strcpy(m_telemetry.tempStatus, "NORMAL");
    strcpy(m_telemetry.modelName, "GoPro HERO");
    strcpy(m_telemetry.lastHexPacket, "--");

    strcpy(m_resStr, "4K");
    strcpy(m_fpsStr, "60 FPS");

    memset(m_rxBuf, 0, sizeof(m_rxBuf));

    m_caps.supportsWkpWake = true;
    m_caps.supportsHilight = true;
    m_caps.supportsKeyReport = false;
    m_caps.supportsTelemetryPush = true;
    m_caps.supportsModeSwitch = true;
    m_caps.modeCount = 3;
    m_caps.availableModes[0] = "VIDEO";
    m_caps.availableModes[1] = "PHOTO";
    m_caps.availableModes[2] = "TIMELAPSE";
}

GoproDriver::~GoproDriver() {}

void GoproDriver::setSavedMac(const uint8_t* mac) {
    if (mac) {
        memcpy(m_savedMac, mac, 6);
        m_hasSavedMac = true;
        snprintf(m_telemetry.cameraMac, sizeof(m_telemetry.cameraMac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

bool GoproDriver::isMatchingAdvertisement(const char* name, const uint8_t* mfgData, size_t mfgLen, const uint8_t* serviceUuid16) {
    if (name && (strstr(name, "GoPro") || strstr(name, "HERO"))) {
        return true;
    }
    // GoPro 16-bit Service UUID is 0xFEA6
    if (serviceUuid16 && serviceUuid16[0] == 0xA6 && serviceUuid16[1] == 0xFE) {
        return true;
    }
    return false;
}

void GoproDriver::configureSecurity() {
    // GoPro requires BLE SMP Pairing/Bonding
}

bool GoproDriver::onConnected(uint16_t connHandle, BleWriteFunc writeFn) {
    m_connHandle = connHandle;
    m_writeFn = writeFn;
    m_telemetry.state = CAM_PAIRED;
    m_rxLen = 0;
    m_rxExpected = 0;
    m_lastKeepAliveMs = millis();
    m_lastQueryMs = millis();
    m_pendingSettingsPush = true;
    m_connectedMs = millis();

    // Open GoPro Push Registration: Query 0x53 to characteristic 0x0076
    // [Len=0x0A][ID=0x53][StatusIDs: 0x06, 0x0A, 0x0D, 0x22, 0x23, 0x36, 0x46, 0x59, 0x60]
    if (m_writeFn) {
        uint8_t pushReg[] = {0x0A, 0x53, 0x06, 0x0A, 0x0D, 0x22, 0x23, 0x36, 0x46, 0x59, 0x60};
        m_writeFn(0x0076, pushReg, sizeof(pushReg), true);
    }

    return true;
}

void GoproDriver::onDisconnected() {
    m_connHandle = 0;
    m_writeFn = nullptr;
    m_telemetry.state = CAM_DISCONNECTED;
    m_telemetry.isRecording = false;
    m_rxLen = 0;
    m_rxExpected = 0;
    m_lastKeepAliveMs = 0;
    m_lastQueryMs = 0;
    m_pendingSettingsPush = false;
    m_connectedMs = 0;
}

void GoproDriver::sendCommand(const uint8_t* cmd, size_t len) {
    if (!m_writeFn || !cmd || len == 0) return;

    // Format hex for log
    char hex[128];
    size_t hlen = 0;
    for (size_t i = 0; i < len && i < 16; i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", cmd[i]);
    }
    if (len > 16) snprintf(hex + hlen, sizeof(hex) - hlen, "...");
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);
    m_telemetry.packetCount++;

    // Write to GoPro Command characteristic (UUID 0072)
    m_writeFn(0x0072, cmd, len, true);
}

void GoproDriver::startRecording() {
    // Open GoPro Shutter On: [03 01 01 01]
    uint8_t cmd[] = {0x03, 0x01, 0x01, 0x01};
    sendCommand(cmd, sizeof(cmd));
    m_telemetry.isRecording = true;
    m_telemetry.state = CAM_RECORDING;
}

void GoproDriver::stopRecording() {
    // Open GoPro Shutter Off: [03 01 01 00]
    uint8_t cmd[] = {0x03, 0x01, 0x01, 0x00};
    sendCommand(cmd, sizeof(cmd));
    m_telemetry.isRecording = false;
    m_telemetry.state = CAM_PAIRED;
}

void GoproDriver::toggleShutter() {
    if (m_telemetry.isRecording) {
        stopRecording();
    } else {
        startRecording();
    }
}

void GoproDriver::tagHilight() {
    // Open GoPro Hilight Tag: [01 18]
    uint8_t cmd[] = {0x01, 0x18};
    sendCommand(cmd, sizeof(cmd));
}

void GoproDriver::locateCamera(bool enable) {
    // Open GoPro Locate Beep (0x16): [03 16 01 <01=on/00=off>]
    uint8_t cmd[] = {0x03, 0x16, 0x01, (uint8_t)(enable ? 0x01 : 0x00)};
    sendCommand(cmd, sizeof(cmd));
}

void GoproDriver::setWifiAp(bool enable) {
    // Open GoPro Wi-Fi AP (0x17): [03 17 01 <01=on/00=off>]
    uint8_t cmd[] = {0x03, 0x17, 0x01, (uint8_t)(enable ? 0x01 : 0x00)};
    sendCommand(cmd, sizeof(cmd));
}

void GoproDriver::setResolution(uint8_t resVal) {
    if (!m_writeFn) return;
    // Settings characteristic (UUID 0074): [03 02 01 <val>]
    uint8_t cmd[] = {0x03, 0x02, 0x01, resVal};
    m_writeFn(0x0074, cmd, sizeof(cmd), true);
}

void GoproDriver::setFps(uint8_t fpsVal) {
    if (!m_writeFn) return;
    // Settings characteristic (UUID 0074): [03 03 01 <val>]
    uint8_t cmd[] = {0x03, 0x03, 0x01, fpsVal};
    m_writeFn(0x0074, cmd, sizeof(cmd), true);
}

void GoproDriver::setStabilization(uint8_t eisVal) {
    if (!m_writeFn) return;
    // Settings characteristic (UUID 0074): [03 87 01 <val>] (Setting 135)
    uint8_t cmd[] = {0x03, 0x87, 0x01, eisVal};
    m_writeFn(0x0074, cmd, sizeof(cmd), true);
}

void GoproDriver::updateResolutionFps() {
    snprintf(m_telemetry.resolutionFps, sizeof(m_telemetry.resolutionFps), "%s / %s", m_resStr, m_fpsStr);
}

void GoproDriver::setMode(uint8_t modeIndex) {
    // Preset Groups to Command char 0x0072:
    // Video:     [04 3E 02 03 E8] (0x03E8 = 1000)
    // Photo:     [04 3E 02 03 E9] (0x03E9 = 1001)
    // Timelapse: [04 3E 02 03 EA] (0x03EA = 1002)
    if (modeIndex == 0) {
        uint8_t cmd[] = {0x04, 0x3E, 0x02, 0x03, 0xE8};
        sendCommand(cmd, sizeof(cmd));
        strcpy(m_telemetry.currentMode, "STANDARD VIDEO");
    } else if (modeIndex == 1) {
        uint8_t cmd[] = {0x04, 0x3E, 0x02, 0x03, 0xE9};
        sendCommand(cmd, sizeof(cmd));
        strcpy(m_telemetry.currentMode, "PHOTO");
    } else if (modeIndex == 2) {
        uint8_t cmd[] = {0x04, 0x3E, 0x02, 0x03, 0xEA};
        sendCommand(cmd, sizeof(cmd));
        strcpy(m_telemetry.currentMode, "TIMELAPSE");
    }
}

void GoproDriver::powerSleep() {
    // Sleep / Power down: [01 05]
    uint8_t cmd[] = {0x01, 0x05};
    sendCommand(cmd, sizeof(cmd));
    m_telemetry.state = CAM_SLEEPING;
    m_telemetry.isRecording = false;
}

void GoproDriver::wakeCamera() {
    m_telemetry.state = CAM_CONNECTING;
}

void GoproDriver::queryStatus() {
    if (!m_writeFn) return;
    // Targeted query for 9 key status values written to Query Write characteristic (UUID 0076)
    uint8_t cmd[] = {0x0A, 0x13, 0x06, 0x0A, 0x0D, 0x22, 0x23, 0x36, 0x46, 0x59, 0x60};

    char hex[96];
    size_t hlen = 0;
    for (size_t i = 0; i < sizeof(cmd) && i < 16; i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", cmd[i]);
    }
    if (sizeof(cmd) > 16) snprintf(hex + hlen, sizeof(hex) - hlen, "...");
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);
    m_telemetry.packetCount++;

    m_writeFn(0x0076, cmd, sizeof(cmd), true);
}

void GoproDriver::sendRawHex(const uint8_t* bytes, size_t len) {
    if (m_writeFn && bytes && len > 0) {
        m_writeFn(0x0072, bytes, len, true);
    }
}

void GoproDriver::onGattNotify(uint16_t handle, const uint8_t* data, size_t len) {
    if (!data || len == 0 || handle != 0x0077) return;
    m_telemetry.packetCount++;

    char hex[96];
    size_t hlen = 0;
    for (size_t i = 0; i < len && i < 16; i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", data[i]);
    }
    if (len > 16) snprintf(hex + hlen, sizeof(hex) - hlen, "...");
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);

    // Multi-packet reassembly for Open GoPro BLE frames
    bool isContinuation = (data[0] & 0x80) != 0;
    if (!isContinuation) {
        // General Header (Start packet)
        uint8_t lenType = (data[0] >> 5) & 0x03;
        size_t expectedLen = 0;
        size_t hdrSize = 1;

        if (lenType == 0) {
            // 5-bit length
            expectedLen = data[0] & 0x1F;
            hdrSize = 1;
        } else if (lenType == 1 && len >= 2) {
            // 13-bit length
            expectedLen = ((size_t)(data[0] & 0x1F) << 8) | data[1];
            hdrSize = 2;
        } else if (lenType == 2 && len >= 3) {
            // 16-bit length
            expectedLen = ((size_t)data[1] << 8) | data[2];
            hdrSize = 3;
        }

        m_rxLen = 0;
        m_rxExpected = expectedLen;
        if (len > hdrSize) {
            size_t chunk = len - hdrSize;
            if (chunk > sizeof(m_rxBuf)) chunk = sizeof(m_rxBuf);
            memcpy(m_rxBuf, data + hdrSize, chunk);
            m_rxLen = chunk;
        }
    } else {
        // Continuation packet: header is 1 byte
        if (len > 1 && m_rxExpected > 0) {
            size_t chunk = len - 1;
            if (m_rxLen + chunk <= sizeof(m_rxBuf)) {
                memcpy(m_rxBuf + m_rxLen, data + 1, chunk);
                m_rxLen += chunk;
            }
        }
    }

    // Process frame when all expected bytes have arrived
    if (m_rxExpected > 0 && m_rxLen >= m_rxExpected) {
        parseStatusResponse(m_rxBuf, m_rxExpected);
        m_rxLen = 0;
        m_rxExpected = 0;
    }
}

void GoproDriver::parseStatusResponse(const uint8_t* data, size_t len) {
    if (!data || len < 2) return;

    uint8_t qId = data[0];
    uint8_t errCode = data[1];

    if (errCode != 0x00) return;

    bool isStatus = (qId == 0x13 || qId == 0x53 || qId == 0x93);
    bool isSetting = (qId == 0x12 || qId == 0x52 || qId == 0x92);
    if (!isStatus && !isSetting) return;

    size_t idx = 2;
    while (idx + 2 <= len) {
        uint8_t id = data[idx++];
        uint8_t valLen = data[idx++];
        if (idx + valLen > len) break;

        const uint8_t* val = &data[idx];

        if (isStatus) {
            switch (id) {
                case 0x06: // System Hot (0=Normal, 1=Overheated)
                    if (valLen >= 1) {
                        if (val[0] == 0) {
                            strcpy(m_telemetry.tempStatus, "NORMAL");
                            m_telemetry.temperatureC = 36;
                        } else {
                            strcpy(m_telemetry.tempStatus, "OVERHEATED");
                            m_telemetry.temperatureC = 75;
                        }
                    }
                    break;

                case 0x0A: // Encoding Active (Recording)
                    if (valLen >= 1) {
                        m_telemetry.isRecording = (val[0] == 1);
                        m_telemetry.state = m_telemetry.isRecording ? CAM_RECORDING : CAM_PAIRED;
                    }
                    break;

                case 0x0D: // Video Progress Counter in seconds
                    if (valLen >= 1) {
                        uint32_t sec = 0;
                        for (size_t k = 0; k < valLen && k < 4; k++) {
                            sec = (sec << 8) | val[k];
                        }
                        m_telemetry.recordTimeSec = sec;
                    }
                    break;

                case 0x22: // Remaining Photos count
                    if (valLen >= 1) {
                        uint32_t photos = 0;
                        for (size_t k = 0; k < valLen && k < 4; k++) {
                            photos = (photos << 8) | val[k];
                        }
                        m_telemetry.remainPhotos = photos;
                    }
                    break;

                case 0x23: // Remaining Video seconds
                    if (valLen >= 1) {
                        uint32_t vsec = 0;
                        for (size_t k = 0; k < valLen && k < 4; k++) {
                            vsec = (vsec << 8) | val[k];
                        }
                        m_telemetry.remainVideoSec = vsec;
                    }
                    break;

                case 0x36: // Remaining Storage Space in KB (up to 64-bit big-endian)
                    if (valLen >= 1) {
                        uint64_t kb = 0;
                        for (size_t k = 0; k < valLen && k < 8; k++) {
                            kb = (kb << 8) | val[k];
                        }
                        m_telemetry.remainCapacityMB = (uint32_t)(kb / 1024ULL);
                    }
                    break;

                case 0x46: // Internal Battery Percentage
                    if (valLen >= 1) {
                        m_telemetry.batteryPercent = val[0];
                    }
                    break;

                case 0x59: // Flatmode
                    if (valLen >= 1) {
                        uint8_t fm = val[0];
                        if (fm == 12) strcpy(m_telemetry.currentMode, "VIDEO");
                        else if (fm == 13) strcpy(m_telemetry.currentMode, "TIMELAPSE VIDEO");
                        else if (fm == 15) strcpy(m_telemetry.currentMode, "LOOPING");
                        else if (fm == 16 || fm == 17) strcpy(m_telemetry.currentMode, "PHOTO");
                        else if (fm == 18 || fm == 19) strcpy(m_telemetry.currentMode, "BURST PHOTO");
                        else if (fm == 20) strcpy(m_telemetry.currentMode, "TIMELAPSE PHOTO");
                        else if (fm == 21) strcpy(m_telemetry.currentMode, "NIGHT LAPSE PHOTO");
                        else if (fm == 24) strcpy(m_telemetry.currentMode, "TIMEWARP VIDEO");
                        else if (fm == 25) strcpy(m_telemetry.currentMode, "LIVE BURST");
                        else if (fm == 26) strcpy(m_telemetry.currentMode, "NIGHT LAPSE VIDEO");
                        else snprintf(m_telemetry.currentMode, sizeof(m_telemetry.currentMode), "MODE %u", fm);
                    }
                    break;

                case 0x60: // Active Preset Group (big-endian uint32)
                    if (valLen >= 1) {
                        uint32_t presetGroup = 0;
                        for (size_t k = 0; k < valLen && k < 4; k++) {
                            presetGroup = (presetGroup << 8) | val[k];
                        }
                        if (presetGroup == 1000) strcpy(m_telemetry.currentMode, "VIDEO");
                        else if (presetGroup == 1001) strcpy(m_telemetry.currentMode, "PHOTO");
                        else if (presetGroup == 1002) strcpy(m_telemetry.currentMode, "TIMELAPSE");
                    }
                    break;

                default:
                    break;
            }
        } else if (isSetting) {
            switch (id) {
                case 0x02: // Setting 2: Resolution (1=4K, 4=2.7K, 9=1080P, 18=4K 4:3, 100=5.3K)
                    if (valLen >= 1) {
                        uint8_t r = val[0];
                        if (r == 1) strcpy(m_resStr, "4K");
                        else if (r == 4) strcpy(m_resStr, "2.7K");
                        else if (r == 9) strcpy(m_resStr, "1080P");
                        else if (r == 18) strcpy(m_resStr, "4K 4:3");
                        else if (r == 100) strcpy(m_resStr, "5.3K");
                        else snprintf(m_resStr, sizeof(m_resStr), "RES %u", r);
                        updateResolutionFps();
                    }
                    break;

                case 0x03: // Setting 3: FPS (0=240, 1=120, 2=100, 5=60, 6=50, 8=30, 9=25, 10=24)
                    if (valLen >= 1) {
                        uint8_t f = val[0];
                        if (f == 0) strcpy(m_fpsStr, "240 FPS");
                        else if (f == 1) strcpy(m_fpsStr, "120 FPS");
                        else if (f == 2) strcpy(m_fpsStr, "100 FPS");
                        else if (f == 5) strcpy(m_fpsStr, "60 FPS");
                        else if (f == 6) strcpy(m_fpsStr, "50 FPS");
                        else if (f == 8) strcpy(m_fpsStr, "30 FPS");
                        else if (f == 9) strcpy(m_fpsStr, "25 FPS");
                        else if (f == 10) strcpy(m_fpsStr, "24 FPS");
                        else snprintf(m_fpsStr, sizeof(m_fpsStr), "%u FPS", f);
                        updateResolutionFps();
                    }
                    break;

                case 0x87: // Setting 135 (0x87): HyperSmooth EIS (0=OFF, 1=ON/LOW, 2=HIGH, 3=BOOST, 4=AUTOBOOST, 100=STANDARD)
                    if (valLen >= 1) {
                        uint8_t s = val[0];
                        if (s == 0) strcpy(m_telemetry.stabilization, "OFF");
                        else if (s == 1) strcpy(m_telemetry.stabilization, "ON/LOW");
                        else if (s == 2) strcpy(m_telemetry.stabilization, "HIGH");
                        else if (s == 3) strcpy(m_telemetry.stabilization, "BOOST");
                        else if (s == 4) strcpy(m_telemetry.stabilization, "AUTOBOOST");
                        else if (s == 100) strcpy(m_telemetry.stabilization, "STANDARD");
                        else snprintf(m_telemetry.stabilization, sizeof(m_telemetry.stabilization), "EIS %u", s);
                    }
                    break;

                default:
                    break;
            }
        }

        idx += valLen;
    }
}

void GoproDriver::updateLoop(uint32_t nowMs) {
    // Delayed Settings Push Registration (250ms after connect): Register for 0x02 (Res), 0x03 (FPS), 0x87 (EIS)
    if (m_pendingSettingsPush && (nowMs - m_connectedMs >= 250)) {
        m_pendingSettingsPush = false;
        if (m_writeFn) {
            uint8_t settingsPushReg[] = {0x04, 0x52, 0x02, 0x03, 0x87};
            m_writeFn(0x0076, settingsPushReg, sizeof(settingsPushReg), true);
        }
    }

    // Keep-Alive: Send [03 5B 01 42] to Settings characteristic (0x0074) every 45s
    if (m_telemetry.state >= CAM_PAIRED && (nowMs - m_lastKeepAliveMs >= 45000)) {
        m_lastKeepAliveMs = nowMs;
        if (m_writeFn) {
            uint8_t keepAlive[] = {0x03, 0x5B, 0x01, 0x42};
            m_writeFn(0x0074, keepAlive, sizeof(keepAlive), true);
        }
    }
}
