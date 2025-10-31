/**
 * Interrupt Debug Example for MCP23009E Library
 *
 * This example helps diagnose interrupt issues by:
 * 1. Checking MCP23009E connectivity
 * 2. Reading all interrupt-related registers
 * 3. Monitoring the interrupt pin state
 * 4. Testing interrupt configuration step by step
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Reset pin
const int RESET_PIN = RST_EXPANDER;
const int INTERRUPT_PIN = INT_EXPANDER;

// Button configuration
const int BUTTON_PIN = MCP23009_BTN_UP;  // GPIO 7

// Track ISR calls
volatile uint32_t isrCallCount = 0;
volatile uint32_t lastIsrTime = 0;

// Simple ISR for testing
void testISR() {
    isrCallCount++;
    lastIsrTime = millis();
}

void printRegisterBinary(const char* name, uint8_t value) {
    Serial.print("  ");
    Serial.print(name);
    Serial.print(": 0x");
    if (value < 0x10) Serial.print("0");
    Serial.print(value, HEX);
    Serial.print(" (");
    Serial.print(value, BIN);
    Serial.println(")");
}

void dumpAllRegisters() {
    Serial.println("\n=== MCP23009E Register Dump ===");
    printRegisterBinary("IODIR  ", mcp.getIODIR());
    printRegisterBinary("IPOL   ", mcp.getIPOL());
    printRegisterBinary("GPINTEN", mcp.getGPINTEN());
    printRegisterBinary("DEFVAL ", mcp.getDEFVAL());
    printRegisterBinary("INTCON ", mcp.getINTCON());
    printRegisterBinary("IOCON  ", mcp.getIOCON().getRegisterValue());
    printRegisterBinary("GPPU   ", mcp.getGPPU());
    printRegisterBinary("INTF   ", mcp.getINTF());
    printRegisterBinary("INTCAP ", mcp.getINTCAP());
    printRegisterBinary("GPIO   ", mcp.getGPIO());
    printRegisterBinary("OLAT   ", mcp.getOLAT());
    Serial.println("==============================\n");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n========================================");
    Serial.println("MCP23009E Interrupt Debug Example");
    Serial.println("========================================\n");
    // Configure MCP23009E reset pin BEFORE I2C initialization
    Serial.println("Configuring MCP23009E reset...");
    pinMode(RST_EXPANDER, OUTPUT);
    digitalWrite(RST_EXPANDER, HIGH);  // Keep reset inactive
    Serial.println("✓ Reset pin configured\n");

    // Initialize I2C
    Serial.println("Step 1: Initializing I2C...");
    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();
    Serial.println("✓ I2C initialized\n");

    // Check MCP23009E connectivity
    Serial.println("Step 2: Checking MCP23009E connectivity...");
    Wire.beginTransmission(MCP23009_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("✗ MCP23009E not found! I2C error: ");
        Serial.println(error);
        while(1) delay(100);
    }
    Serial.println("✓ MCP23009E found\n");

    // Initialize MCP23009E (without interrupt pin first)
    Serial.println("Step 3: Initializing MCP23009E...");
    mcp.begin(RESET_PIN, -1);  // No interrupt pin yet
    Serial.println("✓ MCP23009E initialized\n");

    // Dump initial register state
    Serial.println("Step 4: Initial register state");
    dumpAllRegisters();

    // Configure button
    Serial.println("Step 5: Configuring button (GPIO 7)...");
    mcp.setup(BUTTON_PIN, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    Serial.print("Button state: ");
    Serial.println(mcp.getLevel(BUTTON_PIN) ? "HIGH" : "LOW");
    Serial.println("✓ Button configured\n");

    // Configure IOCON for interrupts
    Serial.println("Step 6: Configuring IOCON register...");
    MCP23009Config config = mcp.getIOCON();
    Serial.print("Current IOCON: 0x");
    Serial.println(config.getRegisterValue(), HEX);

    config.setODR();  // Enable open-drain
    mcp.setIOCON(config);

    config = mcp.getIOCON();
    Serial.print("New IOCON: 0x");
    Serial.println(config.getRegisterValue(), HEX);
    Serial.println("✓ IOCON configured (ODR enabled)\n");

    // Enable interrupt on button
    Serial.println("Step 7: Enabling interrupt on GPIO 7...");
    mcp.interruptOnFalling(BUTTON_PIN, testISR);
    dumpAllRegisters();

    // Configure microcontroller interrupt pin
    Serial.println("Step 8: Configuring microcontroller interrupt pin...");
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.print("INT pin state: ");
    Serial.println(digitalRead(INTERRUPT_PIN) ? "HIGH" : "LOW");

    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), testISR, FALLING);
    Serial.println("✓ Microcontroller interrupt attached\n");

    Serial.println("========================================");
    Serial.println("Setup complete! Monitoring interrupts...");
    Serial.println("Press button UP to test");
    Serial.println("========================================\n");
}

uint32_t lastPrintTime = 0;
uint32_t lastIsrCount = 0;
uint8_t lastButtonState = HIGH;
uint8_t lastIntPinState = HIGH;

void loop() {
    // Process pending interrupts
    bool processed = mcp.processPendingInterrupts();
    if (processed) {
        Serial.println("*** processPendingInterrupts() returned true ***");
    }

    uint32_t currentTime = millis();

    // Read current states
    uint8_t buttonState = mcp.getLevel(BUTTON_PIN);
    uint8_t intPinState = digitalRead(INTERRUPT_PIN);
    uint8_t intf = mcp.getINTF();

    // Detect button state change
    if (buttonState != lastButtonState) {
        Serial.print("[");
        Serial.print(currentTime);
        Serial.print("ms] Button: ");
        Serial.println(buttonState ? "RELEASED" : "PRESSED");
        lastButtonState = buttonState;
    }

    // Detect INT pin state change
    if (intPinState != lastIntPinState) {
        Serial.print("[");
        Serial.print(currentTime);
        Serial.print("ms] INT pin: ");
        Serial.println(intPinState ? "HIGH" : "LOW");
        lastIntPinState = intPinState;

        // If INT goes LOW, dump registers
        if (intPinState == LOW) {
            Serial.println("*** Interrupt detected! ***");
            dumpAllRegisters();
        }
    }

    // Detect ISR calls
    if (isrCallCount != lastIsrCount) {
        Serial.print("[");
        Serial.print(currentTime);
        Serial.print("ms] ISR called! Count: ");
        Serial.println(isrCallCount);
        lastIsrCount = isrCallCount;
    }

    // Periodic status
    if (currentTime - lastPrintTime > 5000) {
        Serial.println("\n--- Status Report ---");
        Serial.print("Button: ");
        Serial.println(buttonState ? "HIGH" : "LOW");
        Serial.print("INT pin: ");
        Serial.println(intPinState ? "HIGH" : "LOW");
        Serial.print("INTF: 0x");
        Serial.println(intf, HEX);
        Serial.print("ISR calls: ");
        Serial.println(isrCallCount);
        Serial.println("--------------------\n");
        lastPrintTime = currentTime;
    }

    delay(10);
}
