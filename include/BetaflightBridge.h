#ifndef BETAFLIGHT_BRIDGE_H
#define BETAFLIGHT_BRIDGE_H

#include <Arduino.h>
#include <WiFi.h>

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

class BetaflightBridge {
public:
    BetaflightBridge();
    ~BetaflightBridge();

    void begin(bool isBridgeMode);
    void loop();

    bool isBridgeMode() const { return m_isBridgeMode; }
    bool isTcpConnected() const { return m_isBridgeMode && m_tcpConnected; }
    bool isFcConnected() const { return m_fcConnected; }
    bool isArmed() const { return m_isArmed; }
    uint32_t getFlightModeFlags() const { return m_flightModeFlags; }

    uint16_t getRcChannel(uint8_t chIdx) const {
        return (chIdx < 12) ? m_rcChannels[chIdx] : 0;
    }

    uint16_t getAuxChannel(uint8_t auxNum) const {
        // AUX1 = index 4, AUX2 = index 5, AUX3 = index 6, AUX4 = index 7
        return (auxNum >= 1 && auxNum <= 8) ? m_rcChannels[3 + auxNum] : 0;
    }

    uint32_t getRxBytes() const { return m_rxBytes; }
    uint32_t getTxBytes() const { return m_txBytes; }
    uint32_t getLastMspResponseMs() const { return m_lastMspResponseMs; }

    void setLogCallback(BfLogCallback cb) { m_logCb = cb; }

private:
    bool m_isBridgeMode;
    bool m_tcpConnected;
    WiFiServer m_tcpServer;
    WiFiClient m_tcpClient;
    BfLogCallback m_logCb;

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
