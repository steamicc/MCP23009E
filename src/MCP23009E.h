/**
 * @file MCP23009E.h
 * @brief Arduino library for MCP23009E I/O Expander
 * @version 1.0.0
 * @author Generated from MicroPython driver
 *
 * This library provides complete control of the MCP23009E I/O expander,
 * including GPIO configuration, interrupts, and Pin-compatible API.
 */

#ifndef MCP23009E_H
#define MCP23009E_H

#include <Arduino.h>
#include <Wire.h>

// IRAM_ATTR is ESP32-specific, define it as empty for other platforms
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ===== Register Addresses =====
#define MCP23009_IODIR      0x00  // I/O Direction Register
#define MCP23009_IPOL       0x01  // Input Polarity Register
#define MCP23009_GPINTEN    0x02  // GPIO Interrupt-on-change Enable
#define MCP23009_DEFVAL     0x03  // Default Value Register
#define MCP23009_INTCON     0x04  // Interrupt Control Register
#define MCP23009_IOCON      0x05  // I/O Expander Configuration
#define MCP23009_GPPU       0x06  // GPIO Pull-up Resistor Register
#define MCP23009_INTF       0x07  // Interrupt Flag Register
#define MCP23009_INTCAP     0x08  // Interrupt Captured Value Register
#define MCP23009_GPIO       0x09  // General Purpose I/O Port Register
#define MCP23009_OLAT       0x0A  // Output Latch Register

// ===== IOCON Register Bits =====
#define MCP23009_IOCON_SEQOP   0x20  // Sequential Operation mode bit
#define MCP23009_IOCON_DISSLW  0x10  // Slew Rate control bit
#define MCP23009_IOCON_ODR     0x04  // Open-Drain output
#define MCP23009_IOCON_INTPOL  0x02  // Interrupt polarity
#define MCP23009_IOCON_INTCC   0x01  // Interrupt Clearing Control

// ===== Default I2C Address =====
#define MCP23009_I2C_ADDR   0x20

// ===== Error Codes =====
enum MCP23009Error {
    MCP23009_OK = 0,              // Success
    MCP23009_ERROR_I2C_WRITE = 1, // I2C write failed
    MCP23009_ERROR_I2C_READ = 2,  // I2C read failed
    MCP23009_ERROR_INVALID_PIN = 3, // Invalid pin number (>7)
    MCP23009_ERROR_TIMEOUT = 4    // I2C timeout
};

// ===== GPIO Configuration =====
#define MCP23009_DIR_OUTPUT  0
#define MCP23009_DIR_INPUT   1

#define MCP23009_NO_PULLUP   0
#define MCP23009_PULLUP      1

#define MCP23009_POL_SAME      0
#define MCP23009_POL_INVERTED  1

#define MCP23009_LOGIC_LOW   0
#define MCP23009_LOGIC_HIGH  1

// ===== Interrupt Configuration =====
#define MCP23009_INTEN_DISABLE  0
#define MCP23009_INTEN_ENABLE   1

#define MCP23009_INTCON_PREVIOUS_STATE  0
#define MCP23009_INTCON_DEFVAL          1

// ===== D-PAD Button Mapping =====
#define MCP23009_BTN_UP     7
#define MCP23009_BTN_DOWN   5
#define MCP23009_BTN_LEFT   6
#define MCP23009_BTN_RIGHT  4

// ===== Forward declarations =====
class MCP23009Pin;
class MCP23009ActiveLowPin;

/**
 * @brief Configuration class for IOCON register
 */
class MCP23009Config {
public:
    MCP23009Config(uint8_t reg = 0x00);

    // SEQOP - Sequential Operation
    MCP23009Config& setSeqOp();
    MCP23009Config& clearSeqOp();
    bool hasSeqOp() const;

    // ODR - Open-Drain output
    MCP23009Config& setODR();
    MCP23009Config& clearODR();
    bool hasODR() const;

    // INTPOL - INT Polarity
    MCP23009Config& setIntPol();
    MCP23009Config& clearIntPol();
    bool hasIntPol() const;

    // INTCC - Interrupt Clearing Control
    MCP23009Config& setIntCC();
    MCP23009Config& clearIntCC();
    bool hasIntCC() const;

    uint8_t getRegisterValue() const;

private:
    uint8_t _reg;
};

/**
 * @brief Main MCP23009E driver class
 */
class MCP23009E {
public:
    /**
     * @brief Constructor
     * @param wire Reference to Wire object (I2C)
     * @param address I2C address (default 0x20)
     */
    MCP23009E(TwoWire& wire = Wire, uint8_t address = MCP23009_I2C_ADDR);

    /**
     * @brief Initialize the MCP23009E
     * @param resetPin Reset pin number (-1 for no reset pin)
     * @param interruptPin Interrupt pin number (-1 for no interrupt)
     */
    void begin(int resetPin = -1, int interruptPin = -1);

    /**
     * @brief Hardware reset
     */
    void reset();

    /**
     * @brief Configure a GPIO pin
     * @param gpx GPIO number (0-7)
     * @param direction Input (1) or Output (0)
     * @param pullup Enable pullup (1) or not (0)
     * @param polarity Same (0) or Inverted (1)
     */
    void setup(uint8_t gpx, uint8_t direction, uint8_t pullup = MCP23009_NO_PULLUP,
               uint8_t polarity = MCP23009_POL_SAME);

    /**
     * @brief Set GPIO output level
     * @param gpx GPIO number (0-7)
     * @param level HIGH (1) or LOW (0)
     */
    void setLevel(uint8_t gpx, uint8_t level);

    /**
     * @brief Read GPIO level
     * @param gpx GPIO number (0-7)
     * @return Level (HIGH or LOW)
     */
    uint8_t getLevel(uint8_t gpx);

    // ===== Interrupt Methods =====
    typedef void (*InterruptCallback)(uint8_t level);
    typedef void (*SimpleCallback)();

    void interruptOnChange(uint8_t gpx, InterruptCallback callback);
    void interruptOnChange(uint8_t gpx, SimpleCallback callback);  // Overload for simple callbacks
    void interruptOnFalling(uint8_t gpx, SimpleCallback callback);
    void interruptOnRaising(uint8_t gpx, SimpleCallback callback);
    void disableInterrupt(uint8_t gpx);

    /**
     * @brief Handle interrupt event (call from ISR or main loop)
     */
    void handleInterrupt();

    /**
     * @brief Check and process pending interrupts (call from main loop)
     * @return true if an interrupt was processed
     */
    bool processPendingInterrupts();

    /**
     * @brief Static ISR handler (automatically configured by begin())
     */
    static void IRAM_ATTR staticISR();

    // ===== Register Access Methods =====
    void setIODIR(uint8_t value);
    uint8_t getIODIR();

    void setIPOL(uint8_t value);
    uint8_t getIPOL();

    void setGPINTEN(uint8_t value);
    uint8_t getGPINTEN();

    void setDEFVAL(uint8_t value);
    uint8_t getDEFVAL();

    void setINTCON(uint8_t value);
    uint8_t getINTCON();

    void setIOCON(const MCP23009Config& config);
    void setIOCON(uint8_t value);
    MCP23009Config getIOCON();

    void setGPPU(uint8_t value);
    uint8_t getGPPU();

    uint8_t getINTF();
    uint8_t getINTCAP();

    void setGPIO(uint8_t value);
    uint8_t getGPIO();

    void setOLAT(uint8_t value);
    uint8_t getOLAT();

    // ===== Error Handling =====
    /**
     * @brief Get last error code
     * @return Error code (MCP23009Error)
     */
    MCP23009Error getLastError() const;

    /**
     * @brief Clear last error
     */
    void clearError();

    /**
     * @brief Check if device is responding on I2C bus
     * @return true if device responds, false otherwise
     */
    bool isConnected();

private:
    TwoWire& _wire;
    uint8_t _address;
    int _resetPin;
    int _interruptPin;

    // Interrupt callback storage
    InterruptCallback _eventsChange[8];
    SimpleCallback _eventsChangeSimple[8];  // For CHANGE mode without level parameter
    SimpleCallback _eventsFall[8];
    SimpleCallback _eventsRise[8];

    // Error tracking
    MCP23009Error _lastError;

    // Static instance for ISR
    static MCP23009E* _isrInstance;
    static volatile bool _interruptPending;

    // Per-GPIO debouncing
    uint32_t _lastInterruptTime[8];
    static const uint32_t DEBOUNCE_DELAY = 0;  // Debounce disabled for testing

    // Private methods
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void sendEnableInterrupt(uint8_t gpx);
    void sendDisableInterrupt(uint8_t gpx);

    static void setBit(uint8_t& reg, uint8_t bit, uint8_t value);
    static uint8_t getBit(uint8_t reg, uint8_t bit);

    friend class MCP23009Pin;
    friend class MCP23009ActiveLowPin;
};

/**
 * @brief Pin-compatible class for MCP23009E GPIO
 *
 * This class provides an Arduino-like Pin interface for MCP23009E GPIO pins.
 * You can use it just like a regular Arduino digital pin.
 *
 * IMPORTANT: The constructor does NOT configure the pin to avoid I2C communication
 * before Wire.begin(). You MUST call pinMode() explicitly after mcp.begin().
 */
class MCP23009Pin {
public:
    /**
     * @brief Constructor
     * @param mcp Reference to MCP23009E instance
     * @param pinNumber GPIO number (0-7)
     * @param mode Pin mode (INPUT, OUTPUT, INPUT_PULLUP) - stored but not applied until pinMode() is called
     */
    MCP23009Pin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode = INPUT);

    /**
     * @brief Initialize or reconfigure the pin
     * @param mode Pin mode
     */
    void pinMode(uint8_t mode);

    /**
     * @brief Write digital value
     * @param value HIGH or LOW
     */
    void digitalWrite(uint8_t value);

    /**
     * @brief Read digital value
     * @return HIGH or LOW
     */
    uint8_t digitalRead();

    /**
     * @brief Set pin HIGH
     */
    void high();

    /**
     * @brief Set pin LOW
     */
    void low();

    /**
     * @brief Toggle pin state
     */
    void toggle();

    /**
     * @brief Attach interrupt
     * @param callback Function to call on interrupt
     * @param mode FALLING, RISING, or CHANGE
     */
    void attachInterrupt(void (*callback)(), int mode);

    /**
     * @brief Detach interrupt
     */
    void detachInterrupt();

    uint8_t getPinNumber() const { return _pinNumber; }

protected:
    MCP23009E& _mcp;
    uint8_t _pinNumber;
    uint8_t _mode;
};

/**
 * @brief Active-Low Pin class for LEDs and similar loads
 *
 * Use this class when connecting LEDs between VCC and GPIO:
 *   3.3V → [LED] → [Resistor 220-330Ω] → GPIO
 *
 * The logic is automatically inverted:
 *   - high() → GPIO LOW → LED ON
 *   - low()  → GPIO HIGH → LED OFF
 */
class MCP23009ActiveLowPin : public MCP23009Pin {
public:
    /**
     * @brief Constructor
     * @param mcp Reference to MCP23009E instance
     * @param pinNumber GPIO number (0-7)
     * @param mode Pin mode (default OUTPUT for LEDs)
     */
    MCP23009ActiveLowPin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode = OUTPUT);

    /**
     * @brief Write digital value (inverted)
     * @param value HIGH or LOW (inverted before writing)
     */
    void digitalWrite(uint8_t value);

    /**
     * @brief Read digital value (inverted)
     * @return HIGH or LOW (inverted after reading)
     */
    uint8_t digitalRead();
};

#endif // MCP23009E_H
