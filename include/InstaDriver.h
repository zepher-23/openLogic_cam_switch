#ifndef INSTA_DRIVER_H
#define INSTA_DRIVER_H

#include "CameraDriver.h"

class InstaDriver : public CameraDriver {
public:
    InstaDriver();
    virtual ~InstaDriver();

    CameraBrand getBrand() const override { return BRAND_INSTA360; }
    const char* getBrandName() const override { return "Insta360 GPS Remote"; }
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
    void sendKeyEvent() override {}
    void setMode(uint8_t modeIndex) override;
    void powerSleep() override;
    void wakeCamera() override;
    void toggleScreen() override;
    void tagHilight() override {}
    void setPushMode(uint8_t pushMode) override {}
    void sendRawHex(const uint8_t* bytes, size_t len) override;

    const CameraTelemetry& getTelemetry() const override { return m_telemetry; }
    void setSavedMac(const uint8_t* mac) override;

    void sendRemoteCommand(uint8_t actionByte, uint8_t paramByte);

private:
    CameraCapabilities m_caps;
    CameraTelemetry m_telemetry;
    BleWriteFunc m_writeFn;
    uint16_t m_connHandle;
    uint8_t m_savedMac[6];
    bool m_hasSavedMac;
};

#endif // INSTA_DRIVER_H
