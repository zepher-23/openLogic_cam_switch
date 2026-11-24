#ifndef BETAFLIGHT_BRIDGE_H
#define BETAFLIGHT_BRIDGE_H

#include <Arduino.h>
#include <algorithm>

#ifndef FC_SERIAL
#define FC_SERIAL Serial1
#endif

#ifndef FC_UART_RX_PIN
#define FC_UART_RX_PIN 20
#endif

#ifndef FC_UART_TX_PIN
#define FC_UART_TX_PIN 21
#endif

typedef void (*BfLogCallback)(const char* tag, const char* msg);
typedef bool (*BfBleTxCallback)(const uint8_t* data, size_t len);

class BetaflightBridge {
public:
    BetaflightBridge();
    ~BetaflightBridge();

    void begin(bool passthroughOnly = false);
    void loop();

    void setBleTxCallback(BfBleTxCallback cb) { m_bleTxCb = cb; }
    void setBleConnected(bool connected) {
        m_bleConnected = connected;
        m_bleResetPending = true;
        if (!connected) {
            m_bleTxChunkMax = 20;
        }
    }
    bool isBleConnected() const { return m_bleConnected; }
    void setBleMtu(uint16_t mtu) {
        m_bleTxChunkMax = std::min<size_t>(mtu > 3 ? mtu - 3 : 20, 244);
    }
    void pushBleRx(const uint8_t* data, size_t len);

    bool isPassthroughOnly() const { return m_passthroughOnly; }
    void setPassthroughOnly(bool p) { m_passthroughOnly = p; }

    bool isFcConnected() const { return m_fcConnected; }
    bool isArmed() const { return m_isArmed; }
    uint32_t getFlightModeFlags() const { return m_flightModeFlags; }

    uint16_t getRcChannel(uint8_t chIdx) const {
        return (chIdx < 12) ? m_rcChannels[chIdx] : 0;
    }

    uint16_t getAuxChannel(uint8_t auxNum) const {
        return (auxNum >= 1 && auxNum <= 8) ? m_rcChannels[3 + auxNum] : 0;
    }

    uint32_t getRxBytes() const { return m_rxBytes; }
    uint32_t getTxBytes() const { return m_txBytes; }
    uint32_t getLastMspResponseMs() const { return m_lastMspResponseMs; }
    uint32_t getLastStatusMs() const { return m_lastStatusMs; }

    void setLogCallback(BfLogCallback cb) { m_logCb = cb; }

private:
    bool m_passthroughOnly;
    bool m_bleConnected;
    BfBleTxCallback m_bleTxCb;
    BfLogCallback m_logCb;

    // Spinlock-protected 512-byte ring buffer for incoming BLE NUS data
    static constexpr size_t BLE_RX_RING_SIZE = 512;
    uint8_t m_bleRxRing[BLE_RX_RING_SIZE];
    size_t m_bleRxHead;
    size_t m_bleRxTail;
    portMUX_TYPE m_bleRxMux;

    // 512-byte ring buffer for outgoing BLE NUS data
    static constexpr size_t BLE_TX_RING_SIZE = 512;
    uint8_t m_bleTxRing[BLE_TX_RING_SIZE];
    size_t m_bleTxHead;
    size_t m_bleTxTail;
    uint32_t m_lastUartRxMs;
    uint32_t m_lastUartRxUs;
    uint32_t m_lastBleTxRetryMs;
    size_t m_bleTxChunkMax;
    volatile bool m_bleResetPending;

    uint32_t m_rxBytes;
    uint32_t m_txBytes;
    uint32_t m_lastUsbRxMs;

    // MSP Diagnostics & Telemetry
    bool m_fcConnected;
    bool m_isArmed;
    uint32_t m_flightModeFlags;
    uint16_t m_rcChannels[12];
    uint32_t m_lastMspPollMs;
    uint32_t m_lastMspResponseMs;
    uint32_t m_lastStatusMs;
    bool m_mspPollToggle;
    bool m_mspLinkLogged;
    bool m_mspTimeoutLogged;

    // Raw UART diagnostics & Inter-byte timeout
    uint32_t m_lastMspByteMs;
    uint32_t m_rawBytesWithoutMsp;
    bool m_rawDiagLogged;
    uint8_t m_firstRawBytes[8];
    uint8_t m_firstRawCount;

    // MSP v1 Stream Parser
    enum MspParserState {
        MSP_IDLE,
        MSP_HEADER_M,
        MSP_HEADER_ARROW,
        MSP_SIZE,
        MSP_CMD,
        MSP_PAYLOAD,
        MSP_CHECKSUM
    };
    MspParserState m_mspState;
    uint8_t m_mspSize;
    uint8_t m_mspCmd;
    uint8_t m_mspPayload[64];
    uint8_t m_mspPayloadIdx;
    uint8_t m_mspChecksum;

    void processMspByte(uint8_t b);
    void handleMspPacket(uint8_t cmd, const uint8_t* payload, uint8_t size);
};

#endif // BETAFLIGHT_BRIDGE_H
