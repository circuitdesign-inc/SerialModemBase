/**
 * @file SerialModemBase.cpp
 * @brief Implementation of common modem logic with non-blocking architecture.
 * @copyright 2026 Circuit Design, Inc.
 */

#include "SerialModemBase.h"
#include <stdio.h>
#include <string.h>

// Define static constexpr members
constexpr char SerialModemBase::CD_WRITE_OK_RESPONSE[];
constexpr size_t SerialModemBase::CD_WRITE_OK_RESPONSE_LEN;
constexpr char SerialModemBase::CD_VAL_ON[];
constexpr char SerialModemBase::CD_VAL_OFF[];
constexpr char SerialModemBase::CD_CMD_WRITE_SUFFIX[];

SerialModemBase::SerialModemBase()
    : _uart(nullptr), _debugStream(nullptr),
      _rxIndex(0), _oneByteBuf(-1), _debugRxNewLine(true),
      _state(State::Idle),
      _queueHead(0), _queueTail(0),
      _lastCmdComplete(true), _lastCmdResult(ModemError::Ok),
      _bTimeout(true), _startTime(0), _timeOutDuration(0)
{
    memset(_rxBuffer, 0, RX_BUFFER_SIZE);
    // Initialize current command with safe defaults
    _currentCmd.type = CommandType::None;
}

void SerialModemBase::setDebugStream(Stream *debugStream)
{
    _debugStream = debugStream;
}

void SerialModemBase::initSerial(Stream &stream)
{
    _uart = &stream;
    clearUnreadByte();
    _debugRxNewLine = true;
    _rxIndex = 0;
    _state = State::Idle;
    _queueHead = 0;
    _queueTail = 0;
    _lastCmdComplete = true;
}

// --- Main State Machine ---

void SerialModemBase::update()
{
    // 1. Always Parse (Non-blocking)
    // Process all available bytes to prevent hardware RX buffer overflow
    while (_uart && (_uart->available() > 0 || _oneByteBuf != -1))
    {
        ModemParseResult parseResult = parse();

        // Handle Async Data Reception immediately (can happen in any state)
        if (parseResult == ModemParseResult::FinishedDrResponse)
        {
            SM_DEBUG_PRINTF("Update: Async DR received. Len=%d\n", (int)_rxIndex);
            onRxDataReceived();
        }

        // Handle command responses within the loop
        if (_state != State::Idle &&
            (parseResult == ModemParseResult::FinishedCmdResponse || parseResult == ModemParseResult::Overflow))
        {
            if (_state == State::WaitingSaveResponse)
            {
                if (_rxIndex == CD_WRITE_OK_RESPONSE_LEN &&
                    strncmp(reinterpret_cast<char *>(_rxBuffer), CD_WRITE_OK_RESPONSE, CD_WRITE_OK_RESPONSE_LEN) == 0)
                {
                    SM_DEBUG_PRINTLN("Update: Save OK, waiting for final response...");
                    startTimeout(_currentCmd.timeoutMs); // Reset timeout for final response
                    _state = State::WaitingResponse;
                }
                else
                {
                    SM_DEBUG_PRINTLN("Update: Unexpected response while waiting for Save.");
                    _lastCmdResult = ModemError::Fail;
                    _lastCmdComplete = true;
                    onCommandComplete(ModemError::Fail);
                    _state = State::Idle;
                }
            }
            else if (_state == State::WaitingResponse)
            {
                SM_DEBUG_PRINTF("Update: Cmd Response received: %s\n", (char *)_rxBuffer);
                _lastCmdResult = ModemError::Ok;
                _lastCmdComplete = true;
                onCommandComplete(ModemError::Ok);
                _state = State::Idle;
            }
        }
    }

    // 2. Timeout Check (only if we didn't just finish a command)
    if (_state != State::Idle && isTimeout())
    {
        SM_DEBUG_PRINTF("Update: Timeout in state %d\n", (int)_state);
        _lastCmdResult = ModemError::Timeout;
        _lastCmdComplete = true;
        onCommandComplete(ModemError::Timeout);
        _state = State::Idle;
    }

    // 3. Queue Handling
    if (_state == State::Idle && !isQueueEmpty())
    {
        // Dequeue
        _currentCmd = _commandQueue[_queueTail];
        _queueTail = (_queueTail + 1) % QUEUE_SIZE;

        // Mark sync state as busy
        _lastCmdComplete = false;
        _lastCmdResult = ModemError::Busy;

        // Send Command
        if (_currentCmd.type == CommandType::DataTx)
        {
            // Send Header
            writeString(_currentCmd.cmdBuffer, true);

            // Short delay to allow modem to process header
            delay(10);

            if (_currentCmd.pPayload && _currentCmd.payloadLen > 0)
            {
                writeData(_currentCmd.pPayload, _currentCmd.payloadLen);
            }

            // Append options/terminator
            if (_currentCmd.suffix[0] != '\0')
            {
                writeString(_currentCmd.suffix, false);
            }
            writeString("\r\n", false);
        }
        else
        {
            // Simple Command
            writeString(_currentCmd.cmdBuffer);
        }

        // Transition
        if (_currentCmd.type == CommandType::NvmSave)
        {
            _state = State::WaitingSaveResponse;
        }
        else
        {
            _state = State::WaitingResponse;
        }
        startTimeout(_currentCmd.timeoutMs);
    }
}

bool SerialModemBase::isIdle() const
{
    return (_state == State::Idle && isQueueEmpty());
}

// --- Command Queue Management ---

bool SerialModemBase::isQueueFull() const
{
    return getQueueCount() >= (QUEUE_SIZE - 1);
}

bool SerialModemBase::isQueueEmpty() const
{
    return _queueHead == _queueTail;
}

uint8_t SerialModemBase::getQueueCount() const
{
    uint8_t h = _queueHead;
    uint8_t t = _queueTail;

    if (h >= t)
        return (h - t);
    return (uint8_t)(QUEUE_SIZE - t + h);
}

// --- Command Queueing ---

ModemError SerialModemBase::enqueueInternal(const char *cmd, CommandType type, const uint8_t *payload, uint8_t len, const char *suffix, uint32_t timeoutMs)
{
    if (isQueueFull())
    {
        SM_DEBUG_PRINTF("Enqueue: Busy! Head=%d, Tail=%d, Count=%d\n", _queueHead, _queueTail, getQueueCount());
        return ModemError::Busy;
    }

    _lastCmdComplete = false;
    ModemCommand &qCmd = _commandQueue[_queueHead];
    qCmd.type = type;
    qCmd.timeoutMs = timeoutMs;
    qCmd.pPayload = payload;
    qCmd.payloadLen = len;

    if (suffix)
    {
        strncpy(qCmd.suffix, suffix, sizeof(qCmd.suffix) - 1);
        qCmd.suffix[sizeof(qCmd.suffix) - 1] = '\0';
    }
    else
    {
        qCmd.suffix[0] = '\0';
    }

    // Copy command string safely
    strncpy(qCmd.cmdBuffer, cmd, sizeof(qCmd.cmdBuffer) - 1);
    qCmd.cmdBuffer[sizeof(qCmd.cmdBuffer) - 1] = '\0';

    _queueHead = (_queueHead + 1) % QUEUE_SIZE;
    return ModemError::Ok;
}

ModemError SerialModemBase::enqueueCommand(const char *cmd, CommandType type, uint32_t timeoutMs)
{
    return enqueueInternal(cmd, type, nullptr, 0, nullptr, timeoutMs);
}

ModemError SerialModemBase::enqueueTxCommand(const char *cmdHeader, const uint8_t *payload, uint8_t len, const char *suffix, uint32_t timeoutMs)
{
    return enqueueInternal(cmdHeader, CommandType::DataTx, payload, len, suffix, timeoutMs);
}

// --- Synchronous Wrappers ---

ModemError SerialModemBase::waitForSyncComplete(uint32_t timeoutMs)
{
    // Wait for the command that was just queued to complete
    // This blocks the user code but keeps the update loop running
    uint32_t start = millis();
    while (!_lastCmdComplete)
    {
        update();
        if (millis() - start > timeoutMs)
        {
            return ModemError::Timeout;
        }
        // Yield to allow other background tasks (ESP32 etc)
        delay(0);
    }
    return _lastCmdResult;
}

// --- Lightweight String Helpers ---
char *SerialModemBase::appendStr(char *dest, const char *src, const char *destEnd)
{
    while (*src && dest < destEnd - 1)
    {
        *dest++ = *src++;
    }
    *dest = '\0';
    return dest;
}

char *SerialModemBase::appendHex2(char *dest, uint8_t val, const char *destEnd)
{
    static const char hexChars[] = "0123456789ABCDEF";
    if (dest < destEnd - 2)
    {
        *dest++ = hexChars[val >> 4];
        *dest++ = hexChars[val & 0x0F];
        *dest = '\0';
    }
    return dest;
}

ModemError SerialModemBase::setByteValue(const char *cmd, uint8_t value, bool save, const char *respPrefix, size_t respLen)
{
    char buf[32];
    char *p = appendStr(buf, buf, cmd);
    p = appendHex2(buf, p, value);
    if (save)
        p = appendStr(buf, p, CD_CMD_WRITE_SUFFIX);
    appendStr(buf, p, "\r\n");

    ModemError err = enqueueCommand(buf, save ? CommandType::NvmSave : CommandType::Simple);
    if (err != ModemError::Ok)
        return err;

    err = waitForSyncComplete(500); // 0.5s sync wait

    if (err == ModemError::Ok)
    {
        uint32_t tmpVal = 0;
        err = parseResponseHex(_rxBuffer, _rxIndex, respPrefix, (uint8_t)(respLen - strlen(respPrefix)), &tmpVal);
        if (err == ModemError::Ok && (uint8_t)tmpVal != value)
        {
            return ModemError::Fail;
        }
    }
    return err;
}

ModemError SerialModemBase::getByteValue(const char *cmd, uint8_t *pValue, const char *respPrefix, size_t respLen)
{
    char buf[16];
    char *p = appendStr(buf, buf, cmd);
    appendStr(buf, p, "\r\n");

    ModemError err = enqueueCommand(buf, CommandType::Simple);
    if (err != ModemError::Ok)
        return err;

    err = waitForSyncComplete(500);

    if (err == ModemError::Ok)
    {
        uint32_t tmpVal = 0;
        err = parseResponseHex(_rxBuffer, _rxIndex, respPrefix, (uint8_t)(respLen - strlen(respPrefix)), &tmpVal);
        if (err == ModemError::Ok && pValue)
        {
            *pValue = static_cast<uint8_t>(tmpVal);
        }
    }
    return err;
}

ModemError SerialModemBase::setBoolValue(const char *baseCmd, bool enabled, bool save, const char *respPrefix)
{
    char buf[32];
    char *p = appendStr(buf, buf, baseCmd);
    p = appendStr(buf, p, enabled ? CD_VAL_ON : CD_VAL_OFF);
    if (save)
        p = appendStr(buf, p, CD_CMD_WRITE_SUFFIX);
    appendStr(buf, p, "\r\n");

    ModemError err = enqueueCommand(buf, save ? CommandType::NvmSave : CommandType::Simple);
    if (err != ModemError::Ok)
        return err;

    err = waitForSyncComplete(500);

    if (err == ModemError::Ok)
    {
        // Verify response
        size_t prefixLen = strlen(respPrefix);
        if (_rxIndex < prefixLen + 2 || strncmp(reinterpret_cast<char *>(_rxBuffer), respPrefix, prefixLen) != 0)
        {
            return ModemError::Fail;
        }
        const char *valPtr = reinterpret_cast<char *>(_rxBuffer) + prefixLen;
        const char *expected = enabled ? CD_VAL_ON : CD_VAL_OFF;
        if (strncmp(valPtr, expected, 2) != 0)
        {
            return ModemError::Fail;
        }
    }
    return err;
}

ModemError SerialModemBase::getBoolValue(const char *cmd, bool *pValue, const char *respPrefix)
{
    char buf[16];
    char *p = appendStr(buf, buf, cmd);
    appendStr(buf, p, "\r\n");

    ModemError err = enqueueCommand(buf, CommandType::Simple);
    if (err != ModemError::Ok)
        return err;

    err = waitForSyncComplete(500);

    if (err == ModemError::Ok)
    {
        size_t prefixLen = strlen(respPrefix);
        if (_rxIndex < prefixLen + 2 || strncmp(reinterpret_cast<char *>(_rxBuffer), respPrefix, prefixLen) != 0)
        {
            return ModemError::Fail;
        }
        const char *valPtr = reinterpret_cast<char *>(_rxBuffer) + prefixLen;
        if (pValue)
            *pValue = (strncmp(valPtr, CD_VAL_ON, 2) == 0);
    }
    return err;
}

ModemError SerialModemBase::sendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs)
{
    if (!command || !responseBuffer)
        return ModemError::InvalidArg;

    ModemError err = enqueueCommand(command, CommandType::Simple, timeoutMs);
    if (err != ModemError::Ok)
        return err;

    err = waitForSyncComplete(timeoutMs + 100);

    if (err == ModemError::Ok)
    {
        if (_rxIndex < bufferSize)
        {
            memcpy(responseBuffer, _rxBuffer, _rxIndex);
            responseBuffer[_rxIndex] = '\0';
        }
        else
        {
            responseBuffer[0] = '\0';
            return ModemError::BufferTooSmall;
        }
    }
    else
    {
        responseBuffer[0] = '\0';
    }
    return err;
}

// --- Low-Level I/O ---

void SerialModemBase::writeString(const char *str, bool printPrefix)
{
    if (!_uart)
        return;
    size_t len = strlen(str);
    if (printPrefix)
        SM_DEBUG_PRINT("TX: ");
    SM_DEBUG_WRITE(reinterpret_cast<const uint8_t *>(str), len);
    _uart->write(reinterpret_cast<const uint8_t *>(str), len);
    _debugRxNewLine = true;
}

void SerialModemBase::writeData(const uint8_t *data, size_t len)
{
    if (!_uart)
        return;
    SM_DEBUG_WRITE(data, len);
    _uart->write(data, len);
}

int SerialModemBase::readByte()
{
    if (!_uart)
        return -1;
    int rcv_int = -1;

    if (_oneByteBuf != -1)
    {
        rcv_int = _oneByteBuf;
        _oneByteBuf = -1;
    }
    else if (_uart->available())
    {
        rcv_int = _uart->read();
    }

    if (rcv_int != -1)
    {
        uint8_t rcv = static_cast<uint8_t>(rcv_int);
        /*
        if (_debugRxNewLine)
        {
            SM_DEBUG_PRINT("RX: ");
            _debugRxNewLine = false;
        }
        // Debug output logic
        if (rcv >= 32 && rcv <= 126)
            SM_DEBUG_WRITE(rcv);
        else if (rcv == '\r')
            SM_DEBUG_WRITE("<CR>");
        else if (rcv == '\n')
        {
            SM_DEBUG_WRITE("<LF>\n");
            _debugRxNewLine = true;
        }
        else
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "<%02X>", rcv);
            SM_DEBUG_WRITE(buf);
        }
        */
        return rcv_int;
    }
    return -1;
}

void SerialModemBase::unreadByte(uint8_t c)
{
    _oneByteBuf = c;
}

void SerialModemBase::clearUnreadByte()
{
    _oneByteBuf = -1;
}

void SerialModemBase::flushGarbage(char keepChar)
{
    if (!_uart)
        return;
    SM_DEBUG_PRINT("Flush: Flushing garbage...");

    if (_oneByteBuf != -1)
    {
        if (_oneByteBuf == keepChar)
            return;
        _oneByteBuf = -1;
    }

    while (_uart->available())
    {
        int c = _uart->read();
        if (c == keepChar)
        {
            unreadByte(static_cast<uint8_t>(c));
            break;
        }
    }
    SM_DEBUG_PRINTLN(" Done.");
}

// --- Timeout Management ---

void SerialModemBase::startTimeout(uint32_t ms)
{
    _bTimeout = false;
    _startTime = millis();
    _timeOutDuration = ms;
}

bool SerialModemBase::isTimeout()
{
    if (!_bTimeout && (millis() - _startTime > _timeOutDuration))
    {
        _bTimeout = true;
    }
    return _bTimeout;
}

// --- Parsing Helpers (Static) ---

bool SerialModemBase::parseHex(const uint8_t *pData, size_t len, uint32_t *pResult)
{
    if (!pData || !pResult)
        return false;
    *pResult = 0;
    for (size_t i = 0; i < len; ++i)
    {
        *pResult <<= 4;
        uint8_t c = pData[i];
        if (c >= '0' && c <= '9')
            *pResult |= (c - '0');
        else if (c >= 'a' && c <= 'f')
            *pResult |= (c - 'a' + 10);
        else if (c >= 'A' && c <= 'F')
            *pResult |= (c - 'A' + 10);
        else
        {
            *pResult = 0;
            return false;
        }
    }
    return true;
}

bool SerialModemBase::parseDec(const uint8_t *pData, size_t len, uint32_t *pResult)
{
    if (!pData || !pResult)
        return false;
    *pResult = 0;
    for (size_t i = 0; i < len; ++i)
    {
        *pResult *= 10;
        uint8_t c = pData[i];
        if (c >= '0' && c <= '9')
            *pResult += (c - '0');
        else
        {
            *pResult = 0;
            return false;
        }
    }
    return true;
}

ModemError SerialModemBase::parseResponseHex(const uint8_t *buffer, size_t length, const char *prefix, uint8_t hexDigits, uint32_t *pResult)
{
    if (!buffer || !prefix || !pResult)
        return ModemError::InvalidArg;
    size_t prefixLen = strlen(prefix);
    if (length < prefixLen + hexDigits)
        return ModemError::Fail;
    if (strncmp(reinterpret_cast<const char *>(buffer), prefix, prefixLen) != 0)
        return ModemError::Fail;
    if (parseHex(buffer + prefixLen, hexDigits, pResult))
        return ModemError::Ok;
    return ModemError::Fail;
}

ModemError SerialModemBase::parseResponseDec(const uint8_t *buffer, size_t length, const char *prefix, const char *suffix, size_t suffixLen, int32_t *pResult)
{
    if (!buffer || !prefix || !pResult)
        return ModemError::InvalidArg;

    size_t prefixLen = strlen(prefix);
    // Check minimum length: prefix + at least 1 digit
    if (length < prefixLen + 1)
        return ModemError::Fail;

    if (strncmp(reinterpret_cast<const char *>(buffer), prefix, prefixLen) != 0)
        return ModemError::Fail;

    // Parse number part
    uint32_t val = 0;
    if (parseDec(buffer + prefixLen, length - prefixLen, &val))
    {
        *pResult = static_cast<int32_t>(val);
        return ModemError::Ok;
    }
    return ModemError::Fail;
}