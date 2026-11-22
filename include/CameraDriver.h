#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BRAND_DJI = 0,
    BRAND_GOPRO = 1,
    BRAND_INSTA360 = 2,
    BRAND_COUNT = 3
} CameraBrand;

typedef enum {
    CAM_DISCONNECTED = 0,
    CAM_CONNECTING = 1,
    CAM_PAIRED = 2,
    CAM_RECORDING = 3,
    CAM_SLEEPING = 4
} CameraState;

typedef struct {
    CameraState state;
    uint8_t batteryPercent;
    bool isRecording;
    uint32_t recordTimeSec;
    uint32_t totalCapacityMB;
    uint32_t remainCapacityMB;
    uint32_t remainPhotos;
    uint32_t remainVideoSec;
    int16_t temperatureC;
    char currentMode[32];
    char resolutionFps[32];
    char stabilization[32];
    char tempStatus[32];
    char modelName[32];
    char lastHexPacket[128];
    uint32_t packetCount;
    char cameraMac[18];
} CameraTelemetry;

typedef struct {
    bool supportsWkpWake;
    bool supportsHilight;
    bool supportsKeyReport;
    bool supportsTelemetryPush;
    bool supportsModeSwitch;
    const char* availableModes[6];
    uint8_t modeCount;
} CameraCapabilities;

#ifdef __cplusplus
} // extern "C"

// Function pointer type for BLE transmission from driver
typedef bool (*BleWriteFunc)(uint16_t handle, const uint8_t* data, size_t len, bool withResponse);
typedef void (*BleLogFunc)(const char* tag, const char* msg);

class CameraDriver {
public:
    virtual ~CameraDriver() {}
    
    virtual CameraBrand getBrand() const = 0;
    virtual const char* getBrandName() const = 0;
    virtual const CameraCapabilities& getCapabilities() const = 0;
    
    // BLE Lifecycle & Matching
    virtual bool isMatchingAdvertisement(const char* name, const uint8_t* mfgData, size_t mfgLen, const uint8_t* serviceUuid16) = 0;
    virtual void configureSecurity() = 0;
    virtual bool onConnected(uint16_t connHandle, BleWriteFunc writeFn) = 0;
    virtual void onDisconnected() = 0;
    virtual void onGattNotify(uint16_t handle, const uint8_t* data, size_t len) = 0;
    virtual void updateLoop(uint32_t nowMs) = 0;

    // Camera Operations
    virtual void startRecording() = 0;
    virtual void stopRecording() = 0;
    virtual void toggleShutter() = 0;
    virtual void sendKeyEvent() = 0;
    virtual void setMode(uint8_t modeIndex) = 0;
    virtual void powerSleep() = 0;
    virtual void wakeCamera() = 0;
    virtual void toggleScreen() {}
    virtual void tagHilight() = 0;
    virtual void setPushMode(uint8_t pushMode) = 0;
    virtual void sendRawHex(const uint8_t* bytes, size_t len) = 0;
    virtual void setStabilization(uint8_t eisMode) {}
    virtual void setResolution(uint8_t resVal) {}
    virtual void setFps(uint8_t fpsVal) {}
    virtual void setWifiAp(bool enable) {}
    virtual void powerNormal() {}
    virtual void sendKeyReport(uint8_t keyCode, uint16_t keyValue) {}
    virtual void sendMockGps() {}
    virtual void locateCamera(bool enable) {}
    virtual void queryStatus() {}
    virtual void sendHeartbeat() {}
    virtual bool shouldDisconnect() const { return false; }

    // Telemetry & State
    virtual const CameraTelemetry& getTelemetry() const = 0;
    virtual void setSavedMac(const uint8_t* mac) = 0;
};

#endif // __cplusplus

#endif // CAMERA_DRIVER_H
