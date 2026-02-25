/**
 * @file SerialModemBase.h
 * @brief Common base class for Circuit Design modems (MLR/MU series).
 * @copyright 2026 Circuit Design, Inc.
 */

#pragma once
#include <Arduino.h>

/**
 * @brief Common error codes for modem operations.
 */
enum class ModemError
{
    Ok,             //!< No error
    Busy,           //!< Driver is busy (Queue full or operation in progress)
    InvalidArg,     //!< Invalid argument
    FailLbt,        //!< Carrier sense failed (LBT)
    Fail,           //!< General failure
    BufferTooSmall, //!< Buffer provided is too small
    Timeout         //!< Operation timed out
};

/**
 * @brief Parser results indicating the status of the last parse step.
 */
enum class ModemParseResult
{
    Parsing,             //!< Still parsing, waiting for more data
    Garbage,             //!< Garbage data received and flushed
    Overflow,            //!< Buffer overflow detected
    FinishedCmdResponse, //!< A standard command response was received
    FinishedDrResponse   //!< A data reception message (DR/DS) was received
};

/**
 * @brief Command types for internal state machine processing.
 */
enum class CommandType
{
    None,
    Simple,  //!< Standard command expecting a response (e.g., @CH01 -> *CH=01)
    NvmSave, //!< Command with /W expecting *WR=PS then final response
    DataTx   //!< Data transmission expecting *DT=XX, then potentially LBT result
};

/**
 * @brief Structure to hold queued commands.
 */
struct ModemCommand
{
    CommandType type;        //!< Type of command
    char cmdBuffer[48];      //!< Command string (header only for DataTx)
    const uint8_t *pPayload; //!< Pointer to payload data (for DataTx)
    uint8_t payloadLen;      //!< Length of payload
    char suffix[48];         //!< Optional suffix for DataTx (e.g., routing options)
    uint32_t timeoutMs;      //!< Timeout for this command
};

// --- Debug Configuration ---
// To enable debug prints, define ENABLE_SERIAL_MODEM_DEBUG in your project
// #define ENABLE_SERIAL_MODEM_DEBUG

#ifdef ENABLE_SERIAL_MODEM_DEBUG
#define SM_DEBUG_PRINT(...) this->dbgPrint(__VA_ARGS__)
#define SM_DEBUG_PRINTLN(...) this->dbgPrintln(__VA_ARGS__)
#define SM_DEBUG_PRINTF(...) this->dbgPrintf(__VA_ARGS__)
#define SM_DEBUG_WRITE(...) this->dbgWrite(__VA_ARGS__)
#else
#define SM_DEBUG_PRINT(...) \
    do                      \
    {                       \
    } while (0)
#define SM_DEBUG_PRINTLN(...) \
    do                        \
    {                         \
    } while (0)
#define SM_DEBUG_PRINTF(...) \
    do                       \
    {                        \
    } while (0)
#define SM_DEBUG_WRITE(...) \
    do                      \
    {                       \
    } while (0)
#endif

/**
 * @brief Base class handling low-level serial I/O, debugging, and async transaction logic.
 */
class SerialModemBase
{
public:
    SerialModemBase();
    virtual ~SerialModemBase() = default;

    /**
     * @brief Initializes the debug stream.
     * @param debugStream Pointer to the Stream for debug output (can be nullptr).
     */
    void setDebugStream(Stream *debugStream);

    /**
     * @brief Main processing loop. Must be called frequently.
     * Handles command queue, transmission, and response parsing.
     */
    void update();

    /**
     * @brief Checks if the internal engine is currently idle (no command processing).
     */
    bool isIdle() const;

    /**
     * @brief Checks if the last processed command has finished.
     * Useful for implementing synchronous wrappers.
     */
    bool isLastCommandComplete() const { return _lastCmdComplete; }

    /**
     * @brief Gets the result of the last processed command.
     */
    ModemError getLastCommandResult() const { return _lastCmdResult; }

    /**
     * @brief Accessor for the shared RX buffer.
     */
    const uint8_t *getRxBuffer() const { return _rxBuffer; }
    uint16_t getRxIndex() const { return _rxIndex; }

    /**
     * @brief Checks if the command queue is full.
     */
    bool isQueueFull() const;

    /**
     * @brief Checks if the command queue is empty.
     */
    bool isQueueEmpty() const;

    /**
     * @brief Gets the number of commands currently in the queue.
     */
    uint8_t getQueueCount() const;

protected:
#ifdef ENABLE_SERIAL_MODEM_DEBUG
    template <typename... Args>
    void dbgPrint(Args... args)
    {
        if (_debugStream)
        {
            _debugStream->print(getLogPrefix());
            _debugStream->print(args...);
        }
    }
    template <typename... Args>
    void dbgPrintln(Args... args)
    {
        if (_debugStream)
        {
            _debugStream->print(getLogPrefix());
            _debugStream->println(args...);
        }
    }
    template <typename... Args>
    void dbgPrintf(Args... args)
    {
        if (_debugStream)
        {
            _debugStream->print(getLogPrefix());
            _debugStream->printf(args...);
        }
    }
    template <typename... Args>
    void dbgWrite(Args... args)
    {
        if (_debugStream)
            _debugStream->write(args...);
    }
#endif

    static constexpr char CD_WRITE_OK_RESPONSE[] = "*WR=PS";
    static constexpr size_t CD_WRITE_OK_RESPONSE_LEN = 6;
    static constexpr char CD_VAL_ON[] = "ON";
    static constexpr char CD_VAL_OFF[] = "OF";
    static constexpr char CD_CMD_WRITE_SUFFIX[] = "/W";

    // --- Initialization ---
    /**
     * @brief Sets the UART stream to use for modem communication.
     * @param stream Reference to the serial stream.
     */
    void initSerial(Stream &stream);

    // --- Command Queueing (For Derived Classes) ---

    /**
     * @brief Enqueues a command for processing. Non-blocking.
     * @return ModemError::Ok if queued, ModemError::Busy if queue is full.
     */
    ModemError enqueueCommand(const char *cmd, CommandType type, uint32_t timeoutMs = 1000);

    /**
     * @brief Enqueues a data transmission command with payload.
     */
    ModemError enqueueTxCommand(const char *cmdHeader, const uint8_t *payload, uint8_t len, const char *suffix = nullptr, uint32_t timeoutMs = 2000);

    // --- Synchronous Wrappers (Implemented via update() loop) ---
    // These replace the old blocking implementations but keep the same signature for API compatibility.

    /**
     * @brief Helper to set a 1-byte value (e.g., @CH0E) and verify the response.
     * * Handles the sequence:
     * 1. Send CMD + Value + (Save ? /W : "")
     * 2. Wait for response
     * 3. If Save, handle intermediate "*WR=PS" and wait for final response
     * 4. Parse final response to verify the value matches.
     * * @param cmd Command string (e.g., "@CH").
     * @param value The byte value to set.
     * @param save Whether to save to NVM (/W).
     * @param respPrefix Expected response prefix (e.g., "*CH=").
     * @param respLen Total length of expected response.
     * @return ModemError::Ok if set and verified successfully.
     */
    ModemError setByteValue(const char *cmd, uint8_t value, bool save, const char *respPrefix, size_t respLen);

    /**
     * @brief Helper to get a 1-byte value.
     * * @param cmd Command string (e.g., "@CH").
     * @param pValue Output pointer for the value.
     * @param respPrefix Expected response prefix.
     * @param respLen Total length of expected response.
     * @return ModemError::Ok on success.
     */
    ModemError getByteValue(const char *cmd, uint8_t *pValue, const char *respPrefix, size_t respLen);

    /**
     * @brief Helper to set a boolean value (ON/OF).
     * * @param baseCmd Base command string (e.g., "@RR"). Appends "ON" or "OF".
     * @param enabled True for ON, False for OFF.
     * @param save Whether to save to NVM (/W).
     * @param respPrefix Expected response prefix (e.g., "*RR=").
     * @return ModemError::Ok if set and verified successfully.
     */
    ModemError setBoolValue(const char *baseCmd, bool enabled, bool save, const char *respPrefix);

    /**
     * @brief Helper to get a boolean value (ON/OF).
     * * @param cmd Command string (e.g., "@RR").
     * @param pValue Output pointer for the value.
     * @param respPrefix Expected response prefix.
     * @return ModemError::Ok on success.
     */
    ModemError getBoolValue(const char *cmd, bool *pValue, const char *respPrefix);

    /**
     * @brief Sends a raw command string and fills the provided buffer with the response.
     * * @param command Command string.
     * @param responseBuffer Output buffer.
     * @param bufferSize Size of output buffer.
     * @param timeoutMs Timeout.
     * @return ModemError::Ok on success.
     */
    ModemError sendRawCommand(const char *command, char *responseBuffer, size_t bufferSize, uint32_t timeoutMs);

    /**
     * @brief Helper to wait for the current command to finish (Pseudo-blocking).
     * Calls update() internally.
     */
    ModemError waitForSyncComplete(uint32_t timeoutMs);

    // --- Low-Level I/O ---
    void writeString(const char *str, bool printPrefix = true);
    void writeData(const uint8_t *data, size_t len);
    int readByte();
    void unreadByte(uint8_t c);
    void clearUnreadByte();
    void flushGarbage(char keepChar = '*');

    // --- Timeout Management ---
    void startTimeout(uint32_t ms);
    bool isTimeout();

    // --- Parsing Helpers ---
    static bool parseHex(const uint8_t *pData, size_t len, uint32_t *pResult);
    static bool parseDec(const uint8_t *pData, size_t len, uint32_t *pResult);
    ModemError parseResponseHex(const uint8_t *buffer, size_t length, const char *prefix, uint8_t hexDigits, uint32_t *pResult);
    ModemError parseResponseDec(const uint8_t *buffer, size_t length, const char *prefix, const char *suffix, size_t suffixLen, int32_t *pResult);

    // --- Virtual Methods (To be implemented by Derived Classes) ---
    /**
     * @brief Main parser state machine step.
     * Must use readByte() and update _rxBuffer / _rxIndex.
     * @return Current status of parsing.
     */
    virtual ModemParseResult parse() = 0;
    virtual void onRxDataReceived() = 0;
    virtual const char *getLogPrefix() const = 0;

    /**
     * @brief Optional hook called when a queued command finishes.
     */
    virtual void onCommandComplete(ModemError result) {}

protected:
    // --- String Helpers (with bounds checking) ---
    static char *appendStr(char *dest, const char *src, const char *destEnd);
    static char *appendHex2(char *dest, uint8_t val, const char *destEnd);

    // Template helpers for automatic size deduction
    template <size_t N>
    static char *appendStr(char (&destBuf)[N], char *currentPtr, const char *src)
    {
        return appendStr(currentPtr, src, destBuf + N);
    }
    template <size_t N>
    static char *appendHex2(char (&destBuf)[N], char *currentPtr, uint8_t val)
    {
        return appendHex2(currentPtr, val, destBuf + N);
    }

    Stream *_uart = nullptr;
    Stream *_debugStream = nullptr;

    // Shared Receive Buffer (Unified)
    static constexpr size_t RX_BUFFER_SIZE = 300;
    uint8_t _rxBuffer[RX_BUFFER_SIZE];
    uint16_t _rxIndex = 0;

    int16_t _oneByteBuf = -1;
    bool _debugRxNewLine = true;

private:
    // --- Internal State Machine ---
    enum class State
    {
        Idle,               //!< No command executing
        Sending,            //!< Sending command string to UART
        WaitingResponse,    //!< Waiting for primary response (e.g., *CH=01)
        WaitingSaveResponse //!< Waiting for *WR=PS before final response
    };
    State _state = State::Idle;

    // --- Command Queue ---
    static constexpr uint8_t QUEUE_SIZE = 8;
    ModemCommand _commandQueue[QUEUE_SIZE];
    volatile uint8_t _queueHead = 0;
    volatile uint8_t _queueTail = 0;

    ModemCommand _currentCmd; // Currently executing command

    // --- Sync/Result State ---
    volatile bool _lastCmdComplete = true;
    volatile ModemError _lastCmdResult = ModemError::Ok;

    // Timeout state
    bool _bTimeout = true;
    uint32_t _startTime = 0;
    uint32_t _timeOutDuration = 0;

    /**
     * @brief Internal helper to enqueue a command, avoiding code duplication.
     */
    ModemError enqueueInternal(const char *cmd, CommandType type, const uint8_t *payload, uint8_t len, const char *suffix, uint32_t timeoutMs);
};