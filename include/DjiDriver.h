#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "CameraDriver.h"

class DjiDriver : public CameraDriver {
public:
    DjiDriver();
    virtual ~DjiDriver();

    CameraBrand getBrand() const override { return BRAND_DJI; }
    const char* getBrandName() const override { return "DJI Osmo Action"; }
    const CameraCapabilities& getCapabilities() const override { return m_caps; }

    bool isMatchingAdvertisement(const char* name, const uint8_t* mfgData, size_t mfgLen, const uint8_t* serviceUuid16) override;
    void configureSecurity() override;
    bool onConnected(uint16_t connHandle, BleWriteFunc writeFn) override;
    void onDisconnected() override;
    void onGattNotify(uint16_t handle, const uint8_t* data, size_t len) override;
    void updateLoop(uint32_t nowMs) override;

    void startRecording() override;
    void stopRecording() override;
    void toggleShutter() override;
    void sendKeyEvent() override;
    void sendKeyReport(uint8_t keyCode, uint16_t keyValue) override;
    void setMode(uint8_t modeIndex) override;
    void powerSleep() override;
    void powerNormal() override;
    void wakeCamera() override;
    void tagHilight() override;
    void setPushMode(uint8_t pushMode) override;
    void sendRawHex(const uint8_t* bytes, size_t len) override;
    void setStabilization(uint8_t eisMode) override;
    void sendGpsData(int32_t date, int32_t time, int32_t lon, int32_t lat, int32_t heightMm, float speedN, float speedE, float speedD);
    void sendMockGps() override;
    void sendPairingHandshake(uint8_t verifyMode = 0);
    bool shouldDisconnect() const override { return m_needsDisconnect; }

    const CameraTelemetry& getTelemetry() const override { return m_telemetry; }
    void setSavedMac(const uint8_t* mac) override;

    // CRC helpers
    static uint16_t calculateCrc16(const uint8_t* data, size_t len);
    static uint32_t calculateCrc32(const uint8_t* data, size_t len);

private:
    void sendPacket(uint8_t cmdType, uint8_t cmdSet, uint8_t cmdId, const uint8_t* payload, size_t payloadLen);
    void sendPacketWithSeq(uint8_t cmdType, uint8_t cmdSet, uint8_t cmdId, uint16_t seq, const uint8_t* payload, size_t payloadLen);
    void parseTelemetryPayload(const uint8_t* payload, size_t len);

    CameraCapabilities m_caps;
    CameraTelemetry m_telemetry;
    BleWriteFunc m_writeFn;
    uint16_t m_connHandle;
    uint16_t m_seq;
    uint8_t m_savedMac[6];
    bool m_hasSavedMac;
    uint32_t m_lastTelemetryMs;
    uint32_t m_lastRetryMs;
    uint32_t m_wakeUntilMs;
    bool m_isWaking;

    // Handshake state machine
    bool m_pendingHandshake;
    uint32_t m_connectedMs;
    bool m_pendingCameraConnAck;
    uint16_t m_pendingCameraConnSeq;
    bool m_postAuthInitNeeded;
    uint32_t m_postAuthMs;
    bool m_needsDisconnect;
    uint32_t m_sleepRequestedMs;
    bool m_sleepPending;
};

#endif // DJI_DRIVER_H
