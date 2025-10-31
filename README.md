# MCP23009E Arduino Library

Complete Arduino library for controlling the MCP23009E I/O Expander with GPIO configuration, interrupts, and Pin-compatible API.

## Features

- ✅ **Full GPIO control**: Configure pins as input/output, read/write levels
- ✅ **Pull-up resistors**: Built-in pull-up support
- ✅ **Interrupt support**: Hardware interrupts with callback system
- ✅ **Pin-compatible API**: `MCP23009Pin` class similar to Arduino's digital pins
- ✅ **Active-low support**: `MCP23009ActiveLowPin` for LEDs and similar loads
- ✅ **Register access**: Low-level register access for advanced usage

## Installation

### Arduino IDE
1. Download this library as ZIP
2. In Arduino IDE: **Sketch** → **Include Library** → **Add .ZIP Library**
3. Select the downloaded ZIP file

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/steamicc/MCP23009E.git
```

## Quick Start

### Basic Usage

```cpp
#include <Wire.h>
#include <MCP23009E.h>

MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

void setup() {
    Serial.begin(115200);
    mcp.begin();  // Initialize I2C and MCP23009E

    // Configure a GPIO as input with pull-up
    mcp.setup(7, MCP23009_DIR_INPUT, MCP23009_PULLUP);

    // Read the level
    uint8_t level = mcp.getLevel(7);
    Serial.println(level ? "HIGH" : "LOW");

    // Configure a GPIO as output
    mcp.setup(0, MCP23009_DIR_OUTPUT);
    mcp.setLevel(0, MCP23009_LOGIC_HIGH);
}

void loop() {
    // Your code here
}
```

### Pin-Compatible API

```cpp
#include <Wire.h>
#include <MCP23009E.h>

MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Create Pin objects (mode will be configured after I2C initialization)
MCP23009Pin button(mcp, 7);
MCP23009Pin led(mcp, 0);

void setup() {
    Wire.begin();
    mcp.begin();

    // Configure pin modes after mcp.begin()
    button.pinMode(INPUT_PULLUP);
    led.pinMode(OUTPUT);

    // Use just like Arduino digital pins!
    led.high();
    if (button.digitalRead() == LOW) {
        led.low();
    }
}

void loop() {
    led.toggle();
    delay(500);
}
```

### Active-Low LEDs

For LEDs connected between VCC and GPIO (recommended configuration):

```
3.3V → [LED] → [Resistor 220-330Ω] → GPIO
```

```cpp
#include <Wire.h>
#include <MCP23009E.h>

MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Create active-low LED pin (logic is automatically inverted)
MCP23009ActiveLowPin led(mcp, 0);

void setup() {
    Wire.begin();
    mcp.begin();

    // Configure pin mode after mcp.begin()
    led.pinMode(OUTPUT);
}

void loop() {
    led.high();   // LED turns ON (GPIO goes LOW)
    delay(500);
    led.low();    // LED turns OFF (GPIO goes HIGH)
    delay(500);
}
```

**Why active-low?** The MCP23009E can sink 25mA per pin but can only source ~1mA. Active-low configuration allows proper LED brightness.

### Error Handling

All I2C operations automatically track errors. Check after critical operations:

```cpp
#include <Wire.h>
#include <MCP23009E.h>

MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

void setup() {
    Serial.begin(115200);
    mcp.begin();

    // Check if device is connected
    if (!mcp.isConnected()) {
        Serial.println("MCP23009E not found!");
        Serial.print("Error code: ");
        Serial.println(mcp.getLastError());
        while(1);  // Halt
    }

    // Configure GPIO and check for errors
    mcp.setup(0, MCP23009_DIR_OUTPUT);

    if (mcp.getLastError() != MCP23009_OK) {
        Serial.println("Configuration failed!");
    }
}

void loop() {
    // Read and check for errors
    uint8_t value = mcp.getLevel(7);

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println(value ? "HIGH" : "LOW");
    } else {
        Serial.println("Read error!");
    }

    delay(1000);
}
```

## API Reference

### MCP23009E Class

#### Constructor
```cpp
MCP23009E(TwoWire& wire = Wire, uint8_t address = MCP23009_I2C_ADDR)
```

#### Main Methods
- `void begin(int resetPin = -1, int interruptPin = -1)` - Initialize the device
- `void reset()` - Hardware reset (requires reset pin)
- `void setup(uint8_t gpx, uint8_t direction, uint8_t pullup, uint8_t polarity)` - Configure GPIO
- `void setLevel(uint8_t gpx, uint8_t level)` - Set output level
- `uint8_t getLevel(uint8_t gpx)` - Read input level

#### Error Handling Methods
- `MCP23009Error getLastError()` - Get last I2C error code
- `void clearError()` - Clear error status
- `bool isConnected()` - Check if device is responding on I2C bus

Error codes:
- `MCP23009_OK (0)` - Success
- `MCP23009_ERROR_I2C_WRITE (1)` - I2C write failed
- `MCP23009_ERROR_I2C_READ (2)` - I2C read failed
- `MCP23009_ERROR_INVALID_PIN (3)` - Invalid pin number
- `MCP23009_ERROR_TIMEOUT (4)` - I2C timeout

#### Interrupt Methods
- `void interruptOnChange(uint8_t gpx, InterruptCallback callback)` - Register change callback
- `void interruptOnFalling(uint8_t gpx, SimpleCallback callback)` - Register falling edge callback
- `void interruptOnRaising(uint8_t gpx, SimpleCallback callback)` - Register rising edge callback
- `void disableInterrupt(uint8_t gpx)` - Disable interrupts on GPIO
- `bool processPendingInterrupts()` - **Call this in loop()** to process pending interrupts (returns true if processed)
- `void handleInterrupt()` - Low-level interrupt handler (usually not called directly)

#### Register Access
- `setIODIR()`, `getIODIR()` - I/O Direction Register
- `setGPPU()`, `getGPPU()` - Pull-up Register
- `setGPIO()`, `getGPIO()` - GPIO Register
- And many more...

### MCP23009Pin Class

#### Constructor
```cpp
MCP23009Pin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode = INPUT)
```

#### Methods
- `void pinMode(uint8_t mode)` - Set pin mode (INPUT, OUTPUT, INPUT_PULLUP)
- `void digitalWrite(uint8_t value)` - Write HIGH or LOW
- `uint8_t digitalRead()` - Read pin state
- `void high()` - Set pin HIGH
- `void low()` - Set pin LOW
- `void toggle()` - Toggle pin state
- `void attachInterrupt(void (*callback)(), int mode)` - Attach interrupt
- `void detachInterrupt()` - Detach interrupt

### MCP23009ActiveLowPin Class

Inherits from `MCP23009Pin` with inverted logic for active-low loads.

#### Constructor
```cpp
MCP23009ActiveLowPin(MCP23009E& mcp, uint8_t pinNumber, uint8_t mode = OUTPUT)
```

All methods are identical to `MCP23009Pin`, but logic is automatically inverted.

## Examples

The library includes several examples:

### Basic Examples
- **BasicUsage** - Read D-PAD buttons and print states
- **LED_ActiveLow** - Control LEDs in active-low configuration with effects
- **ErrorHandling** - Demonstrates I2C error detection and handling

### Interrupt Examples
- **InterruptBasic** - Simple interrupt handling with one button (FALLING/RISING)
- **InterruptMultipleButtons** - Handle interrupts from multiple buttons (CHANGE mode)
- **InterruptWithPin** - Use Pin API with interrupts and LED control

## Hardware Considerations

### I2C Address
Default address is `0x20`. Can be changed via hardware pins A0-A2.

### Current Limits
- **Sink current (IOL)**: 25 mA max per pin
- **Source current (IOH)**: ~1 mA per pin

**Recommendation**: Use active-low configuration for LEDs (connect between VCC and GPIO).

### Pull-ups
The MCP23009E has built-in 100kΩ pull-up resistors that can be enabled per pin.

## Compatibility

- **Arduino**: All boards with I2C support
- **ESP32**: Tested and working
- **ESP8266**: Tested and working
- **STM32**: Tested and working
- **Others**: Should work on any platform with Wire library

## License

This library is released under the MIT License. See LICENSE file for details.

## Credits

Based on the MicroPython driver for MCP23009E.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on GitHub.
