#include "InstaDriver.h"
#include <cstdio>
#include <cstring>
#include <Arduino.h>

InstaDriver::InstaDriver()
    : m_writeFn(nullptr)
    , m_connHandle(0)
    , m_hasSavedMac(false)
{
    memset(&m_telemetry, 0, sizeof(m_telemetry));
    m_telemetry.state = CAM_DISCONNECTED;
    strcpy(m_telemetry.currentMode, "REMOTE STANDBY");
    strcpy(m_telemetry.resolutionFps, "COMMAND ONLY");
    strcpy(m_telemetry.stabilization, "FLOWSTATE");
    strcpy(m_telemetry.tempStatus, "NORMAL");
    strcpy(m_telemetry.modelName, "Insta360 GPS Remote");
    strcpy(m_telemetry.lastHexPacket, "--");

    m_caps.supportsWkpWake = false;
    m_caps.supportsHilight = false;
    m_caps.supportsKeyReport = false;
    m_caps.supportsTelemetryPush = false;
    m_caps.supportsModeSwitch = true;
    m_caps.modeCount = 1;
    m_caps.availableModes[0] = "CYCLE MODE";
}

InstaDriver::~InstaDriver() {}

void InstaDriver::setSavedMac(const uint8_t* mac) {
    if (mac) {
        memcpy(m_savedMac, mac, 6);
        m_hasSavedMac = true;
        snprintf(m_telemetry.cameraMac, sizeof(m_telemetry.cameraMac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

bool InstaDriver::isMatchingAdvertisement(const char* name, const uint8_t* mfgData, size_t mfgLen, const uint8_t* serviceUuid16) {
    // When acting as peripheral, matching is handled by BLE server advertisements
    return false;
}

void InstaDriver::configureSecurity() {
    // Insta360 GPS Remote Peripheral uses standard BLE security
}

bool InstaDriver::onConnected(uint16_t connHandle, BleWriteFunc writeFn) {
    m_connHandle = connHandle;
    m_writeFn = writeFn;
    m_telemetry.state = CAM_PAIRED;
    strcpy(m_telemetry.currentMode, "REMOTE CONNECTED");
    strcpy(m_telemetry.tempStatus, "NORMAL");
    return true;
}

void InstaDriver::onDisconnected() {
    m_connHandle = 0;
    m_writeFn = nullptr;
    m_telemetry.state = CAM_DISCONNECTED;
    m_telemetry.isRecording = false;
    strcpy(m_telemetry.currentMode, "REMOTE STANDBY");
}

void InstaDriver::sendRemoteCommand(uint8_t actionByte, uint8_t paramByte) {
    // Official Insta360 GPS Remote 9-byte packet format:
    // [0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, <action>, <param>]
    uint8_t frame[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, actionByte, paramByte};

    char hex[64];
    size_t hlen = 0;
    for (size_t i = 0; i < sizeof(frame); i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", frame[i]);
    }
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);
    m_telemetry.packetCount++;

    if (m_writeFn) {
        // Send via characteristic 0xCE82 notification
        m_writeFn(0xCE82, frame, sizeof(frame), false);
    }
}

void InstaDriver::startRecording() {
    // Shutter Toggle: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00}
    sendRemoteCommand(0x02, 0x00);
}

void InstaDriver::stopRecording() {
    // Shutter Toggle: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00}
    sendRemoteCommand(0x02, 0x00);
}

void InstaDriver::toggleShutter() {
    // Shutter Toggle: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00}
    sendRemoteCommand(0x02, 0x00);
}

void InstaDriver::setMode(uint8_t modeIndex) {
    // Mode Cycle: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x01, 0x00}
    sendRemoteCommand(0x01, 0x00);
}

void InstaDriver::toggleScreen() {
    // Screen Toggle: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x00}
    sendRemoteCommand(0x00, 0x00);
}

void InstaDriver::powerSleep() {
    // Power Off: {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x03}
    sendRemoteCommand(0x00, 0x03);
    m_telemetry.state = CAM_SLEEPING;
}

void InstaDriver::wakeCamera() {
    // Wake via connect
    m_telemetry.state = CAM_CONNECTING;
}

void InstaDriver::sendRawHex(const uint8_t* bytes, size_t len) {
    if (m_writeFn && bytes && len > 0) {
        m_writeFn(0xCE82, bytes, len, false);
    }
}

void InstaDriver::onGattNotify(uint16_t handle, const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    m_telemetry.packetCount++;

    char hex[64];
    size_t hlen = 0;
    for (size_t i = 0; i < len && i < 16; i++) {
        hlen += snprintf(hex + hlen, sizeof(hex) - hlen, "%02X ", data[i]);
    }
    strncpy(m_telemetry.lastHexPacket, hex, sizeof(m_telemetry.lastHexPacket) - 1);
}

void InstaDriver::updateLoop(uint32_t nowMs) {
    // Command-only mode: no periodic poll needed
}
