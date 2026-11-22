#ifndef GOPRO_DRIVER_H
#define GOPRO_DRIVER_H

#include "CameraDriver.h"

class GoproDriver : public CameraDriver {
public:
    GoproDriver();
    virtual ~GoproDriver();

    CameraBrand getBrand() const override { return BRAND_GOPRO; }
    const char* getBrandName() const override { return "GoPro HERO"; }
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
    void tagHilight() override;
    void setPushMode(uint8_t pushMode) override {}
    void sendRawHex(const uint8_t* bytes, size_t len) override;
    void locateCamera(bool enable) override;
    void setWifiAp(bool enable) override;
    void setStabilization(uint8_t eisVal) override;
    void setResolution(uint8_t resVal) override;
    void setFps(uint8_t fpsVal) override;
    void queryStatus() override;

    const CameraTelemetry& getTelemetry() const override { return m_telemetry; }
    void setSavedMac(const uint8_t* mac) override;

private:
    void sendCommand(const uint8_t* cmd, size_t len);
    void parseStatusResponse(const uint8_t* data, size_t len);
    void updateResolutionFps();

    CameraCapabilities m_caps;
    CameraTelemetry m_telemetry;
    BleWriteFunc m_writeFn;
    uint16_t m_connHandle;
    uint8_t m_savedMac[6];
    bool m_hasSavedMac;
    uint32_t m_lastQueryMs;
    uint32_t m_lastKeepAliveMs;
    char m_resStr[12];
    char m_fpsStr[12];
    bool m_pendingSettingsPush;
    uint32_t m_connectedMs;
    uint8_t m_rxBuf[512];
    size_t m_rxLen;
    size_t m_rxExpected;
};

#endif // GOPRO_DRIVER_H
