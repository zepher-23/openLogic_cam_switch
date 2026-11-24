#include "BetaflightBridge.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

BetaflightBridge::BetaflightBridge()
    : m_passthroughOnly(false)
    , m_bleConnected(false)
    , m_bleTxCb(nullptr)
    , m_logCb(nullptr)
    , m_bleRxHead(0)
    , m_bleRxTail(0)
    , m_bleRxMux(portMUX_INITIALIZER_UNLOCKED)
    , m_bleTxHead(0)
    , m_bleTxTail(0)
    , m_lastUartRxMs(0)
    , m_lastUartRxUs(0)
    , m_lastBleTxRetryMs(0)
    , m_bleTxChunkMax(20)
    , m_bleResetPending(false)
    , m_rxBytes(0)
    , m_txBytes(0)
    , m_lastUsbRxMs(0)
    , m_fcConnected(false)
    , m_isArmed(false)
    , m_flightModeFlags(0)
    , m_lastMspPollMs(0)
    , m_lastMspResponseMs(0)
    , m_lastStatusMs(0)
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
    memset(m_bleRxRing, 0, sizeof(m_bleRxRing));
    memset(m_bleTxRing, 0, sizeof(m_bleTxRing));
    memset(m_rcChannels, 0, sizeof(m_rcChannels));
    memset(m_mspPayload, 0, sizeof(m_mspPayload));
    memset(m_firstRawBytes, 0, sizeof(m_firstRawBytes));
}

BetaflightBridge::~BetaflightBridge() {
}

void BetaflightBridge::pushBleRx(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    portENTER_CRITICAL(&m_bleRxMux);
    for (size_t i = 0; i < len; i++) {
        size_t nextHead = (m_bleRxHead + 1) % BLE_RX_RING_SIZE;
        if (nextHead != m_bleRxTail) {
            m_bleRxRing[m_bleRxHead] = data[i];
            m_bleRxHead = nextHead;
        } else {
            break; // Drop if ring buffer is full
        }
    }
    portEXIT_CRITICAL(&m_bleRxMux);
}

void BetaflightBridge::begin(bool passthroughOnly) {
    m_passthroughOnly = passthroughOnly;

    // Initialize Hardware UART for Flight Controller
    FC_SERIAL.setRxBufferSize(1024);
    FC_SERIAL.begin(115200, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);

    if (m_logCb) {
        char buf[80];
        snprintf(buf, sizeof(buf), "FC UART active on RX=GPIO%d, TX=GPIO%d @ 115200 baud",
                 FC_UART_RX_PIN, FC_UART_TX_PIN);
        m_logCb("UART", buf);
    }
}

void BetaflightBridge::loop() {
    uint32_t now = millis();

    // Reset rings and drain residual UART bytes if connection state changed
    if (m_bleResetPending) {
        m_bleResetPending = false;
        m_bleTxHead = m_bleTxTail = 0;
        m_lastBleTxRetryMs = 0;
        portENTER_CRITICAL(&m_bleRxMux);
        m_bleRxHead = m_bleRxTail = 0;
        portEXIT_CRITICAL(&m_bleRxMux);
        if (m_bleConnected) {
            while (FC_SERIAL.available() > 0) {
                FC_SERIAL.read();
            }
        }
    }

    // Drain BLE RX ring -> FC UART on loopTask
    uint8_t bleRxBuf[128];
    size_t bleRxCount = 0;
    portENTER_CRITICAL(&m_bleRxMux);
    while (m_bleRxTail != m_bleRxHead && bleRxCount < sizeof(bleRxBuf)) {
        bleRxBuf[bleRxCount++] = m_bleRxRing[m_bleRxTail];
        m_bleRxTail = (m_bleRxTail + 1) % BLE_RX_RING_SIZE;
    }
    portEXIT_CRITICAL(&m_bleRxMux);
    if (bleRxCount > 0) {
        FC_SERIAL.write(bleRxBuf, bleRxCount);
        m_rxBytes += bleRxCount;
    }

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

    // 2. Read FC UART -> Forward to BLE & USB Serial & Feed MSP Parser
    uint8_t uartBuf[256];
    size_t uartCount = 0;
    while (FC_SERIAL.available() > 0 && uartCount < sizeof(uartBuf)) {
        uartBuf[uartCount++] = (uint8_t)FC_SERIAL.read();
    }
    if (uartCount > 0) {
        m_txBytes += uartCount;

        if (m_bleConnected && m_bleTxCb) {
            for (size_t i = 0; i < uartCount; i++) {
                size_t nextHead = (m_bleTxHead + 1) % BLE_TX_RING_SIZE;
                if (nextHead != m_bleTxTail) {
                    m_bleTxRing[m_bleTxHead] = uartBuf[i];
                    m_bleTxHead = nextHead;
                } else break;
            }
            m_lastUartRxMs = now;
            m_lastUartRxUs = micros();
        }

        bool isUsbConfigActive = (m_lastUsbRxMs > 0 && (now - m_lastUsbRxMs < 1500));

        // Forward raw FC UART to USB Serial ONLY when PC Betaflight Configurator is actively communicating via USB
        if (isUsbConfigActive && Serial && Serial.availableForWrite() >= (int)uartCount) {
            Serial.write(uartBuf, uartCount);
        }

        // Feed MSP Parser only when not in passthrough, BLE bridge, or USB config
        if (!m_passthroughOnly && !m_bleConnected && !isUsbConfigActive) {
            for (size_t i = 0; i < uartCount; i++) {
                processMspByte(uartBuf[i]);
            }
        }

        // Raw UART diagnostics: track bytes arriving with no valid MSP
        if (!m_fcConnected && !m_passthroughOnly && !m_bleConnected && !isUsbConfigActive) {
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
        } else if (m_fcConnected) {
            m_rawBytesWithoutMsp = 0;
            m_firstRawCount = 0;
            m_rawDiagLogged = false;
        }
    }

    // BLE TX Flush Step: Forward UART data to BLE with zero artificial latency
    while (m_bleConnected && m_bleTxCb) {
        size_t buffered = (m_bleTxHead >= m_bleTxTail) ? (m_bleTxHead - m_bleTxTail) : (BLE_TX_RING_SIZE - m_bleTxTail + m_bleTxHead);
        if (buffered == 0) {
            break;
        }
        if (m_lastBleTxRetryMs > 0 && (now - m_lastBleTxRetryMs < 1)) {
            break;
        }
        // Only wait to coalesce when packet is smaller than chunk max and more bytes are actively arriving
        if (buffered < m_bleTxChunkMax && (FC_SERIAL.available() > 0 || (uint32_t)(micros() - m_lastUartRxUs) < 1200UL)) {
            break;
        }
        size_t toSend = std::min<size_t>(buffered, m_bleTxChunkMax);
        uint8_t chunk[244];
        for (size_t i = 0; i < toSend; i++) {
            chunk[i] = m_bleTxRing[(m_bleTxTail + i) % BLE_TX_RING_SIZE];
        }
        if (m_bleTxCb(chunk, toSend)) {
            m_bleTxTail = (m_bleTxTail + toSend) % BLE_TX_RING_SIZE;
            m_lastBleTxRetryMs = 0;
        } else {
            m_lastBleTxRetryMs = now;
            break;
        }
    }

    // 4. Alternate MSP Polling: MSP_STATUS (101) & MSP_RC (105) every 100ms (5 Hz each)
    // Run ONLY when !m_passthroughOnly && !m_bleConnected && !isUsbConfigActive
    bool isUsbConfigActive = (m_lastUsbRxMs > 0 && (now - m_lastUsbRxMs < 1500));
    if (!m_passthroughOnly && !m_bleConnected && !isUsbConfigActive) {
        if (now - m_lastMspPollMs >= 100) {
            m_lastMspPollMs = now;
            m_mspPollToggle = !m_mspPollToggle;
            if (m_mspPollToggle) {
                // MSP_STATUS (101): $M<\0\x65\x65
                static const uint8_t mspStatusReq[6] = {0x24, 0x4D, 0x3C, 0x00, 0x65, 0x65};
                FC_SERIAL.write(mspStatusReq, sizeof(mspStatusReq));
            } else {
                // MSP_RC (105): $M<\0\x69\x69
                static const uint8_t mspRcReq[6] = {0x24, 0x4D, 0x3C, 0x00, 0x69, 0x69};
                FC_SERIAL.write(mspRcReq, sizeof(mspRcReq));
            }
        }

        // FC Connection Watchdog
        now = millis();
        if (m_fcConnected && (int32_t)(now - m_lastMspResponseMs) > 1500) {
            m_fcConnected = false;
        }

        if (m_lastMspResponseMs > 0 && (int32_t)(now - m_lastMspResponseMs) > 3500) {
            if (!m_mspTimeoutLogged) {
                m_mspTimeoutLogged = true;
                if (m_logCb) {
                    m_logCb("MSP", "Warning: FC telemetry silent (>3.5s); recording continues");
                }
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
            } else {
                m_mspState = MSP_IDLE;
            }
            break;
        case MSP_SIZE:
            m_mspSize = b;
            m_mspChecksum = b;
            m_mspPayloadIdx = 0;
            m_mspState = MSP_CMD;
            break;
        case MSP_CMD:
            m_mspCmd = b;
            m_mspChecksum ^= b;
            if (m_mspSize > 0) {
                m_mspState = MSP_PAYLOAD;
            } else {
                m_mspState = MSP_CHECKSUM;
            }
            break;
        case MSP_PAYLOAD:
            if (m_mspPayloadIdx < sizeof(m_mspPayload)) {
                m_mspPayload[m_mspPayloadIdx] = b;
            }
            m_mspPayloadIdx++;
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
    }
}

void BetaflightBridge::handleMspPacket(uint8_t cmd, const uint8_t* payload, uint8_t size) {
    m_lastMspResponseMs = millis();
    m_mspTimeoutLogged = false;

    if (!m_fcConnected) {
        m_fcConnected = true;
        if (m_logCb) {
            m_logCb("MSP", "FC link online (valid telemetry response received)");
        }
    }

    // 101 = MSP_STATUS, 150 = MSP_STATUS_EX
    if ((cmd == 101 || cmd == 150) && size >= 10) {
        m_lastStatusMs = millis();
        m_flightModeFlags = (uint32_t)payload[6] |
                           ((uint32_t)payload[7] << 8) |
                           ((uint32_t)payload[8] << 16) |
                           ((uint32_t)payload[9] << 24);

        m_isArmed = (m_flightModeFlags & 0x00000001UL) != 0;

        if (!m_mspLinkLogged) {
            m_mspLinkLogged = true;
            if (m_logCb) {
                char dbg[96];
                snprintf(dbg, sizeof(dbg), "MSP link verified: cmd=%u, size=%u, flags=0x%08lX (armBit=%d)",
                         cmd, size, (unsigned long)m_flightModeFlags, (int)m_isArmed);
                m_logCb("MSP", dbg);
            }
        }
    }
    // 105 = MSP_RC
    else if (cmd == 105 && size >= 8) {
        uint8_t numChannels = size / 2;
        if (numChannels > 12) numChannels = 12;
        for (uint8_t i = 0; i < numChannels; i++) {
            m_rcChannels[i] = (uint16_t)payload[i * 2] | ((uint16_t)payload[i * 2 + 1] << 8);
        }
    }
}
