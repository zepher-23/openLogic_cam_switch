#include "BetaflightBridge.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

BetaflightBridge::BetaflightBridge()
    : m_isBridgeMode(false)
    , m_tcpConnected(false)
    , m_tcpServer(5761)
    , m_tcpClient()
    , m_logCb(nullptr)
    , m_rxBytes(0)
    , m_txBytes(0)
    , m_lastUsbRxMs(0)
    , m_fcConnected(false)
    , m_isArmed(false)
    , m_flightModeFlags(0)
    , m_lastMspPollMs(0)
    , m_lastMspResponseMs(0)
    , m_mspPollToggle(false)
    , m_mspLinkLogged(false)
    , m_mspTimeoutLogged(false)
    , m_lastMspByteMs(0)
    , m_rawBytesWithoutMsp(0)
    , m_rawDiagLogged(false)
    , m_firstRawCount(0)
    , m_mspState(MSP_IDLE)
    , m_mspSize(0)
    , m_mspCmd(0)
    , m_mspPayloadIdx(0)
    , m_mspChecksum(0)
{
    memset(m_rcChannels, 0, sizeof(m_rcChannels));
    memset(m_mspPayload, 0, sizeof(m_mspPayload));
    memset(m_firstRawBytes, 0, sizeof(m_firstRawBytes));
}

BetaflightBridge::~BetaflightBridge() {
    if (m_tcpClient) {
        m_tcpClient.stop();
    }
}

void BetaflightBridge::begin(bool isBridgeMode) {
    m_isBridgeMode = isBridgeMode;

    // Initialize Hardware UART for Flight Controller
    FC_SERIAL.setRxBufferSize(1024);
    FC_SERIAL.begin(115200, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);

    if (m_logCb) {
        char buf[80];
        snprintf(buf, sizeof(buf), "FC UART active on RX=GPIO%d, TX=GPIO%d @ 115200 baud",
                 FC_UART_RX_PIN, FC_UART_TX_PIN);
        m_logCb("UART", buf);
    }

    if (m_isBridgeMode) {
        m_tcpServer.begin();
        m_tcpServer.setNoDelay(true);
        if (m_logCb) {
            m_logCb("BRIDGE", "Dedicated TCP Bridge listening on 192.168.4.1:5761");
        }
    }
}

void BetaflightBridge::loop() {
    uint32_t now = millis();

    if (m_isBridgeMode) {
        // --- Dedicated TCP Bridge Mode ---
        // Accept new TCP client on port 5761
        if (m_tcpServer.hasClient()) {
            WiFiClient newClient = m_tcpServer.accept();
            if (!m_tcpClient || !m_tcpClient.connected()) {
                m_tcpClient = newClient;
                m_tcpClient.setNoDelay(true);
                if (m_logCb) {
                    m_logCb("BRIDGE", "Configurator connected via TCP (port 5761)");
                }
            } else {
                newClient.stop(); // Only 1 active configurator session at a time
            }
        }

        m_tcpConnected = (m_tcpClient && m_tcpClient.connected());

        // Drain TCP Client -> FC UART
        if (m_tcpClient && m_tcpClient.connected()) {
            uint8_t tcpBuf[256];
            int tcpAvail = m_tcpClient.available();
            if (tcpAvail > 0) {
                int toRead = std::min<int>(tcpAvail, (int)sizeof(tcpBuf));
                int bytesRead = m_tcpClient.read(tcpBuf, toRead);
                if (bytesRead > 0) {
                    FC_SERIAL.write(tcpBuf, bytesRead);
                    m_rxBytes += bytesRead;
                }
            }
        }

        // Drain USB Serial (COM7 PC) -> FC UART
        uint8_t usbBuf[128];
        size_t usbCount = 0;
        while (Serial.available() > 0 && usbCount < sizeof(usbBuf)) {
            usbBuf[usbCount++] = (uint8_t)Serial.read();
        }
        if (usbCount > 0) {
            m_lastUsbRxMs = now;
            FC_SERIAL.write(usbBuf, usbCount);
            m_rxBytes += usbCount;
        }

        // Drain FC UART -> TCP Client & USB Serial
        uint8_t uartBuf[256];
        size_t uartCount = 0;
        while (FC_SERIAL.available() > 0 && uartCount < sizeof(uartBuf)) {
            uartBuf[uartCount++] = (uint8_t)FC_SERIAL.read();
        }
        if (uartCount > 0) {
            if (m_tcpClient && m_tcpClient.connected()) {
                m_tcpClient.write(uartBuf, uartCount);
            }
            if (Serial && Serial.availableForWrite() >= (int)uartCount) {
                Serial.write(uartBuf, uartCount);
            }
            m_txBytes += uartCount;
        }

        // Pause internal MSP polling in bridge mode
        return;
    }

    // --- Normal Camera Mode ---
    // 1. Drain USB Serial (COM7 PC) -> FC UART
    uint8_t usbBuf[128];
    size_t usbCount = 0;
    while (Serial.available() > 0 && usbCount < sizeof(usbBuf)) {
        usbBuf[usbCount++] = (uint8_t)Serial.read();
    }
    if (usbCount > 0) {
        m_lastUsbRxMs = now;
        FC_SERIAL.write(usbBuf, usbCount);
        m_rxBytes += usbCount;
    }

    // 2. Read FC UART -> Feed MSP Parser & Passthrough to USB Serial
    uint8_t uartBuf[128];
    size_t uartCount = 0;
    while (FC_SERIAL.available() > 0 && uartCount < sizeof(uartBuf)) {
        uartBuf[uartCount++] = (uint8_t)FC_SERIAL.read();
    }
    if (uartCount > 0) {
        // Raw UART diagnostics: track bytes arriving with no valid MSP
        if (!m_fcConnected) {
            for (size_t i = 0; i < uartCount && m_firstRawCount < 8; i++) {
                m_firstRawBytes[m_firstRawCount++] = uartBuf[i];
            }
            m_rawBytesWithoutMsp += uartCount;
            if (m_rawBytesWithoutMsp > 50 && !m_rawDiagLogged) {
                m_rawDiagLogged = true;
                if (m_logCb) {
                    char diagMsg[128];
                    snprintf(diagMsg, sizeof(diagMsg),
                             "UART warning: %lu bytes received with no valid MSP (first: %02X %02X %02X %02X %02X %02X %02X %02X) - check baud/pins/DisplayPort",
                             (unsigned long)m_rawBytesWithoutMsp,
                             m_firstRawBytes[0], m_firstRawBytes[1], m_firstRawBytes[2], m_firstRawBytes[3],
                             m_firstRawBytes[4], m_firstRawBytes[5], m_firstRawBytes[6], m_firstRawBytes[7]);
                    m_logCb("UART", diagMsg);
                }
            }
        }

        for (size_t i = 0; i < uartCount; i++) {
            processMspByte(uartBuf[i]);
        }
        if (Serial && Serial.availableForWrite() >= (int)uartCount) {
            Serial.write(uartBuf, uartCount);
        }
        m_txBytes += uartCount;
    }

    // 3. Alternate MSP Polling: MSP_STATUS (101) & MSP_RC (105) every 100ms (5 Hz each)
    // Only poll when PC USB Configurator is not actively writing to the UART
    bool isUsbConfigActive = (m_lastUsbRxMs > 0 && (now - m_lastUsbRxMs < 1500));
    if (!isUsbConfigActive) {
        if (now - m_lastMspPollMs >= 100) {
            m_lastMspPollMs = now;
            m_mspPollToggle = !m_mspPollToggle;
            if (m_mspPollToggle) {
                // MSP_STATUS (101): $M<\0\x65\x65
                static const uint8_t mspStatusReq[6] = {0x24, 0x4D, 0x3C, 0x00, 0x65, 0x65};
                FC_SERIAL.write(mspStatusReq, sizeof(mspStatusReq));
                m_rxBytes += sizeof(mspStatusReq);
            } else {
                // MSP_RC (105): $M<\0\x69\x69
                static const uint8_t mspRcReq[6] = {0x24, 0x4D, 0x3C, 0x00, 0x69, 0x69};
                FC_SERIAL.write(mspRcReq, sizeof(mspRcReq));
                m_rxBytes += sizeof(mspRcReq);
            }
        }
    }

    // 4. FC Connection Watchdog
    if (m_fcConnected && (now - m_lastMspResponseMs > 1500)) {
        m_fcConnected = false;
    }

    if (now - m_lastMspResponseMs > 3500 && m_lastMspResponseMs > 0) {
        if (!m_mspTimeoutLogged) {
            m_mspTimeoutLogged = true;
            if (m_logCb) {
                m_logCb("MSP", "Warning: FC telemetry silent (>3.5s); recording continues");
            }
        }
    }
}

void BetaflightBridge::processMspByte(uint8_t b) {
    uint32_t now = millis();
    if (m_mspState != MSP_IDLE && (now - m_lastMspByteMs > 50)) {
        m_mspState = MSP_IDLE;
    }
    m_lastMspByteMs = now;

    switch (m_mspState) {
        case MSP_IDLE:
            if (b == '$') {
                m_mspState = MSP_HEADER_M;
            }
            break;
        case MSP_HEADER_M:
            if (b == 'M') {
                m_mspState = MSP_HEADER_ARROW;
            } else if (b == '$') {
                m_mspState = MSP_HEADER_M;
            } else {
                m_mspState = MSP_IDLE;
            }
            break;
        case MSP_HEADER_ARROW:
            if (b == '>') {
                m_mspState = MSP_SIZE;
            } else if (b == '$') {
                m_mspState = MSP_HEADER_M;
            } else {
                m_mspState = MSP_IDLE;
            }
            break;
        case MSP_SIZE:
            m_mspSize = b;
            m_mspChecksum = b;
            m_mspState = MSP_CMD;
            break;
        case MSP_CMD:
            m_mspCmd = b;
            m_mspChecksum ^= b;
            m_mspPayloadIdx = 0;
            if (m_mspSize == 0) {
                m_mspState = MSP_CHECKSUM;
            } else if (m_mspSize <= sizeof(m_mspPayload)) {
                m_mspState = MSP_PAYLOAD;
            } else {
                m_mspState = MSP_IDLE;
            }
            break;
        case MSP_PAYLOAD:
            m_mspPayload[m_mspPayloadIdx++] = b;
            m_mspChecksum ^= b;
            if (m_mspPayloadIdx >= m_mspSize) {
                m_mspState = MSP_CHECKSUM;
            }
            break;
        case MSP_CHECKSUM:
            if (b == m_mspChecksum) {
                handleMspPacket(m_mspCmd, m_mspPayload, m_mspSize);
            }
            m_mspState = MSP_IDLE;
            break;
        default:
            m_mspState = MSP_IDLE;
            break;
    }
}

void BetaflightBridge::handleMspPacket(uint8_t cmd, const uint8_t* payload, uint8_t size) {
    bool wasConnected = m_fcConnected;
    m_lastMspResponseMs = millis();
    m_fcConnected = true;
    m_mspTimeoutLogged = false;
    m_rawBytesWithoutMsp = 0;
    m_rawDiagLogged = false;
    m_firstRawCount = 0;

    // 101 = MSP_STATUS, 150 = MSP_STATUS_EX
    if ((cmd == 101 || cmd == 150) && size >= 10) {
        m_flightModeFlags = (uint32_t)payload[6] |
                           ((uint32_t)payload[7] << 8) |
                           ((uint32_t)payload[8] << 16) |
                           ((uint32_t)payload[9] << 24);

        m_isArmed = (m_flightModeFlags & 0x00000001UL) != 0;
    }

    // 105 = MSP_RC
    if (cmd == 105 && size >= 16) {
        size_t numChannels = std::min<size_t>(size / 2, 12);
        for (size_t i = 0; i < numChannels; i++) {
            m_rcChannels[i] = (uint16_t)payload[2 * i] | ((uint16_t)payload[2 * i + 1] << 8);
        }
    }

    // Re-log whenever m_fcConnected transitions from false to true (not just once per boot)
    if (!wasConnected) {
        if (m_logCb) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "FC link online: cmd=%u arm=%s flags=0x%08lX AUX1=%u AUX2=%u AUX3=%u AUX4=%u",
                     cmd, m_isArmed ? "YES" : "NO", (unsigned long)m_flightModeFlags,
                     m_rcChannels[4], m_rcChannels[5], m_rcChannels[6], m_rcChannels[7]);
            m_logCb("MSP", msg);
        }
    }
}
