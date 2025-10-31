/**
 * @file MCP23009E.cpp
 * @brief Implementation of MCP23009E I/O Expander library
 */

#include "MCP23009E.h"

// ===== MCP23009Config Implementation =====

MCP23009Config::MCP23009Config(uint8_t reg) : _reg(reg & 0b00100111) {}

MCP23009Config& MCP23009Config::setSeqOp() {
    _reg |= 0x20;
    return *this;
}

MCP23009Config& MCP23009Config::clearSeqOp() {
    _reg &= ~0x20;
    return *this;
}

bool MCP23009Config::hasSeqOp() const {
    return (_reg & 0x20) != 0;
}

MCP23009Config& MCP23009Config::setODR() {
    _reg |= 0x04;
    return *this;
}

MCP23009Config& MCP23009Config::clearODR() {
    _reg &= ~0x04;
    return *this;
}

bool MCP23009Config::hasODR() const {
    return (_reg & 0x04) != 0;
}

MCP23009Config& MCP23009Config::setIntPol() {
    _reg |= 0x02;
    return *this;
}

MCP23009Config& MCP23009Config::clearIntPol() {
    _reg &= ~0x02;
    return *this;
}

bool MCP23009Config::hasIntPol() const {
    return (_reg & 0x02) != 0;
}

MCP23009Config& MCP23009Config::setIntCC() {
    _reg |= 0x01;
    return *this;
}

MCP23009Config& MCP23009Config::clearIntCC() {
    _reg &= ~0x01;
    return *this;
}

bool MCP23009Config::hasIntCC() const {
    return (_reg & 0x01) != 0;
}

uint8_t MCP23009Config::getRegisterValue() const {
    return _reg;
}

// ===== MCP23009E Implementation =====

MCP23009E::MCP23009E(TwoWire& wire, uint8_t address)
    : _wire(wire), _address(address), _resetPin(-1), _interruptPin(-1), _lastError(MCP23009_OK) {
    // Initialize callback arrays to nullptr
    for (int i = 0; i < 8; i++) {
        _eventsChange[i] = nullptr;
        _eventsChangeSimple[i] = nullptr;
        _eventsFall[i] = nullptr;
        _eventsRise[i] = nullptr;
    }
}

void MCP23009E::begin(int resetPin, int interruptPin) {
    _resetPin = resetPin;
    _interruptPin = interruptPin;

    // Configure reset pin if provided
    if (_resetPin >= 0) {
        pinMode(_resetPin, OUTPUT);
        reset();
    }

    // Configure interrupt pin if provided
    if (_interruptPin >= 0) {
        pinMode(_interruptPin, INPUT);
    }

    // Initialize I2C
    _wire.begin();
}

void MCP23009E::reset() {
    if (_resetPin >= 0) {
        digitalWrite(_resetPin, LOW);
        delay(5);
        digitalWrite(_resetPin, HIGH);
        delay(10);
    }
}

void MCP23009E::setup(uint8_t gpx, uint8_t direction, uint8_t pullup, uint8_t polarity) {
    if (gpx > 7) return;

    // Read current registers
    uint8_t iodir = readRegister(MCP23009_IODIR);
    uint8_t gppu = readRegister(MCP23009_GPPU);
    uint8_t ipol = readRegister(MCP23009_IPOL);

    // Modify bits
    setBit(iodir, gpx, direction);
    setBit(gppu, gpx, pullup);
    setBit(ipol, gpx, polarity);

    // Write modified registers
    writeRegister(MCP23009_IODIR, iodir);
    writeRegister(MCP23009_GPPU, gppu);
    writeRegister(MCP23009_IPOL, ipol);
}

void MCP23009E::setLevel(uint8_t gpx, uint8_t level) {
    if (gpx > 7) return;

    // Check if pin is configured as output
    uint8_t iodir = readRegister(MCP23009_IODIR);
    if (getBit(iodir, gpx) == MCP23009_DIR_INPUT) return;

    // Modify GPIO register
    uint8_t gpio = readRegister(MCP23009_GPIO);
    setBit(gpio, gpx, level);
    writeRegister(MCP23009_GPIO, gpio);
}

uint8_t MCP23009E::getLevel(uint8_t gpx) {
    if (gpx > 7) return MCP23009_LOGIC_LOW;

    uint8_t gpio = readRegister(MCP23009_GPIO);
    return getBit(gpio, gpx);
}

// ===== Interrupt Methods =====

void MCP23009E::sendEnableInterrupt(uint8_t gpx) {
    uint8_t gpinten = readRegister(MCP23009_GPINTEN);
    uint8_t intcon = readRegister(MCP23009_INTCON);

    setBit(gpinten, gpx, MCP23009_INTEN_ENABLE);
    setBit(intcon, gpx, MCP23009_INTCON_PREVIOUS_STATE);

    writeRegister(MCP23009_GPINTEN, gpinten);
    writeRegister(MCP23009_INTCON, intcon);
}

void MCP23009E::sendDisableInterrupt(uint8_t gpx) {
    uint8_t gpinten = readRegister(MCP23009_GPINTEN);
    uint8_t intcon = readRegister(MCP23009_INTCON);
    uint8_t defval = readRegister(MCP23009_DEFVAL);

    setBit(gpinten, gpx, MCP23009_INTEN_DISABLE);
    setBit(intcon, gpx, MCP23009_INTCON_PREVIOUS_STATE);
    setBit(defval, gpx, MCP23009_LOGIC_LOW);

    writeRegister(MCP23009_GPINTEN, gpinten);
    writeRegister(MCP23009_INTCON, intcon);
    writeRegister(MCP23009_DEFVAL, defval);
}

void MCP23009E::interruptOnChange(uint8_t gpx, InterruptCallback callback) {
    if (gpx > 7) return;
    sendEnableInterrupt(gpx);
    _eventsChange[gpx] = callback;
    _eventsChangeSimple[gpx] = nullptr;  // Clear simple callback
}

void MCP23009E::interruptOnChange(uint8_t gpx, SimpleCallback callback) {
    if (gpx > 7) return;
    sendEnableInterrupt(gpx);
    _eventsChange[gpx] = nullptr;  // Clear complex callback
    _eventsChangeSimple[gpx] = callback;
}

void MCP23009E::interruptOnFalling(uint8_t gpx, SimpleCallback callback) {
    if (gpx > 7) return;
    sendEnableInterrupt(gpx);
    _eventsFall[gpx] = callback;
}

void MCP23009E::interruptOnRaising(uint8_t gpx, SimpleCallback callback) {
    if (gpx > 7) return;
    sendEnableInterrupt(gpx);
    _eventsRise[gpx] = callback;
}

void MCP23009E::disableInterrupt(uint8_t gpx) {
    if (gpx > 7) return;
    sendDisableInterrupt(gpx);
    _eventsChange[gpx] = nullptr;
    _eventsChangeSimple[gpx] = nullptr;
    _eventsFall[gpx] = nullptr;
    _eventsRise[gpx] = nullptr;
}

void MCP23009E::handleInterrupt() {
    // Read interrupt configuration
    MCP23009Config iocon = getIOCON();

    // Read interrupt flags
    uint8_t intf = readRegister(MCP23009_INTF);

    // Read GPIO state (clears interrupt)
    uint8_t state;
    if (iocon.hasIntCC()) {
        state = readRegister(MCP23009_INTCAP);
    } else {
        state = readRegister(MCP23009_GPIO);
    }

    // Process each GPIO that generated an interrupt
    for (uint8_t i = 0; i < 8; i++) {
        if (getBit(intf, i)) {
            uint8_t level = getBit(state, i);

            // Call appropriate callbacks
            if (level == MCP23009_LOGIC_HIGH) {
                if (_eventsRise[i] != nullptr) {
                    _eventsRise[i]();
                }
            } else {
                if (_eventsFall[i] != nullptr) {
                    _eventsFall[i]();
                }
            }

            if (_eventsChange[i] != nullptr) {
                _eventsChange[i](level);
            }
            if (_eventsChangeSimple[i] != nullptr) {
                _eventsChangeSimple[i]();
            }
        }
    }
}

// ===== Register Access Methods =====

void MCP23009E::setIODIR(uint8_t value) {
    writeRegister(MCP23009_IODIR, value);
}

uint8_t MCP23009E::getIODIR() {
    return readRegister(MCP23009_IODIR);
}

void MCP23009E::setIPOL(uint8_t value) {
    writeRegister(MCP23009_IPOL, value);
}

uint8_t MCP23009E::getIPOL() {
    return readRegister(MCP23009_IPOL);
}

void MCP23009E::setGPINTEN(uint8_t value) {
    writeRegister(MCP23009_GPINTEN, value);
}

uint8_t MCP23009E::getGPINTEN() {
    return readRegister(MCP23009_GPINTEN);
}

void MCP23009E::setDEFVAL(uint8_t value) {
    writeRegister(MCP23009_DEFVAL, value);
}

uint8_t MCP23009E::getDEFVAL() {
    return readRegister(MCP23009_DEFVAL);
}

void MCP23009E::setINTCON(uint8_t value) {
    writeRegister(MCP23009_INTCON, value);
}

uint8_t MCP23009E::getINTCON() {
    return readRegister(MCP23009_INTCON);
}

void MCP23009E::setIOCON(const MCP23009Config& config) {
    writeRegister(MCP23009_IOCON, config.getRegisterValue());
}

void MCP23009E::setIOCON(uint8_t value) {
    writeRegister(MCP23009_IOCON, value);
}

MCP23009Config MCP23009E::getIOCON() {
    return MCP23009Config(readRegister(MCP23009_IOCON));
}

void MCP23009E::setGPPU(uint8_t value) {
    writeRegister(MCP23009_GPPU, value);
}

uint8_t MCP23009E::getGPPU() {
    return readRegister(MCP23009_GPPU);
}

uint8_t MCP23009E::getINTF() {
    return readRegister(MCP23009_INTF);
}

uint8_t MCP23009E::getINTCAP() {
    return readRegister(MCP23009_INTCAP);
}

void MCP23009E::setGPIO(uint8_t value) {
    writeRegister(MCP23009_GPIO, value);
}

uint8_t MCP23009E::getGPIO() {
    return readRegister(MCP23009_GPIO);
}

void MCP23009E::setOLAT(uint8_t value) {
    writeRegister(MCP23009_OLAT, value);
}

uint8_t MCP23009E::getOLAT() {
    return readRegister(MCP23009_OLAT);
}

// ===== Private Methods =====

void MCP23009E::writeRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    uint8_t error = _wire.endTransmission();

    if (error != 0) {
        _lastError = MCP23009_ERROR_I2C_WRITE;
    } else {
        _lastError = MCP23009_OK;
    }
}

uint8_t MCP23009E::readRegister(uint8_t reg) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    uint8_t error = _wire.endTransmission();

    if (error != 0) {
        _lastError = MCP23009_ERROR_I2C_WRITE;
        return 0;
    }

    uint8_t received = _wire.requestFrom(_address, (uint8_t)1);
    if (received != 1) {
        _lastError = MCP23009_ERROR_I2C_READ;
        return 0;
    }

    uint8_t value = _wire.read();
    _lastError = MCP23009_OK;
    return value;
}

void MCP23009E::setBit(uint8_t& reg, uint8_t bit, uint8_t value) {
    if (value == 0) {
        reg &= ~(1 << bit);
    } else {
        reg |= (1 << bit);
    }
}

uint8_t MCP23009E::getBit(uint8_t reg, uint8_t bit) {
    return (reg & (1 << bit)) ? 1 : 0;
}

// ===== MCP23009Pin Implementation =====

MCP23009Pin::MCP23009Pin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode)
    : _mcp(mcp), _pinNumber(pinNumber), _mode(mode) {
    pinMode(mode);
}

void MCP23009Pin::pinMode(uint8_t mode) {
    _mode = mode;

    if (mode == INPUT_PULLUP) {
        _mcp.setup(_pinNumber, INPUT, MCP23009_PULLUP);
    } else {
        _mcp.setup(_pinNumber, mode, MCP23009_NO_PULLUP);
    }
}

void MCP23009Pin::digitalWrite(uint8_t value) {
    _mcp.setLevel(_pinNumber, value);
}

uint8_t MCP23009Pin::digitalRead() {
    return _mcp.getLevel(_pinNumber);
}

void MCP23009Pin::high() {
    digitalWrite(HIGH);
}

void MCP23009Pin::low() {
    digitalWrite(LOW);
}

void MCP23009Pin::toggle() {
    digitalWrite(1 - digitalRead());
}

void MCP23009Pin::attachInterrupt(void (*callback)(), int mode) {
    switch (mode) {
        case FALLING:
            _mcp.interruptOnFalling(_pinNumber, callback);
            break;
        case RISING:
            _mcp.interruptOnRaising(_pinNumber, callback);
            break;
        case CHANGE:
            _mcp.interruptOnChange(_pinNumber, callback);  // Use SimpleCallback overload
            break;
    }
}

void MCP23009Pin::detachInterrupt() {
    _mcp.disableInterrupt(_pinNumber);
}

// ===== MCP23009ActiveLowPin Implementation =====

MCP23009ActiveLowPin::MCP23009ActiveLowPin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode)
    : MCP23009Pin(mcp, pinNumber, mode) {
    // Initialize to HIGH (LED off)
    if (mode == OUTPUT) {
        digitalWrite(LOW);  // LOW means LED off in active-low
    }
}

void MCP23009ActiveLowPin::digitalWrite(uint8_t value) {
    // Invert the value
    MCP23009Pin::digitalWrite(1 - value);
}

uint8_t MCP23009ActiveLowPin::digitalRead() {
    // Invert the read value
    return 1 - MCP23009Pin::digitalRead();
}

// ===== Error Handling Methods =====

MCP23009Error MCP23009E::getLastError() const {
    return _lastError;
}

void MCP23009E::clearError() {
    _lastError = MCP23009_OK;
}

bool MCP23009E::isConnected() {
    _wire.beginTransmission(_address);
    uint8_t error = _wire.endTransmission();

    if (error == 0) {
        _lastError = MCP23009_OK;
        return true;
    } else {
        _lastError = MCP23009_ERROR_I2C_WRITE;
        return false;
    }
}
