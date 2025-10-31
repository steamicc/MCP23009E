/**
 * Interrupt with Pin API Example for MCP23009E Library
 *
 * This example demonstrates using the Pin-compatible API with interrupts.
 * The MCP23009Pin class provides an Arduino-like interface for GPIO pins
 * including interrupt support with attachInterrupt/detachInterrupt.
 *
 * Hardware Setup:
 * - MCP23009E connected via I2C
 * - Button on GPIO 7 (UP button)
 * - LED on GPIO 0 (active-low configuration)
 * - MCP23009E INT pin connected to microcontroller GPIO (INT_EXPANDER)
 *
 * Behavior:
 * - Press button: Toggle LED and increment counter
 * - Long press (>2s): Reset counter
 *
 * Note: The interrupt pin configuration and ISR attachment are now handled
 *       automatically by the library when you pass the interruptPin to begin().
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Reset pin
const int RESET_PIN = RST_EXPANDER;
const int INTERRUPT_PIN = INT_EXPANDER;

// Create Pin objects (mode will be configured in setup after I2C initialization)
MCP23009Pin button(mcp, MCP23009_BTN_UP);
MCP23009ActiveLowPin led(mcp, 0);

// Button state
volatile bool buttonEvent = false;
volatile uint32_t buttonPressTime = 0;
const uint32_t LONG_PRESS_DURATION = 2000;  // 2 seconds

uint32_t toggleCount = 0;
bool ledState = false;

// Button press callback (uses Simple mode - no level parameter)
void onButtonPressed() {
    buttonPressTime = millis();
}

// Button release callback
void onButtonReleased() {
    uint32_t pressDuration = millis() - buttonPressTime;

    // Signal main loop to handle the event
    buttonEvent = true;

    // Check for long press (>2 seconds)
    if (pressDuration > LONG_PRESS_DURATION) {
        // Reset counter on long press
        toggleCount = 0;
        Serial.println("\n>>> LONG PRESS detected - Counter reset!");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Initializing I2C...");
    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();

    Serial.println("========================================");
    Serial.println("MCP23009E Pin API with Interrupt Example");
    Serial.println("========================================\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN, INTERRUPT_PIN);

    // Configure pin modes after I2C initialization
    button.pinMode(INPUT_PULLUP);
    led.pinMode(OUTPUT);

    // Start with LED off
    led.low();

    // Attach interrupts to button using Pin API
    // Note: attachInterrupt uses SimpleCallback (no level parameter)
    // The interrupt pin and ISR are automatically configured by begin()
    button.attachInterrupt(onButtonPressed, FALLING);
    button.attachInterrupt(onButtonReleased, RISING);

    Serial.println("✓ Configuration complete");
    Serial.println("\nUsage:");
    Serial.println("- Short press: Toggle LED and increment counter");
    Serial.println("- Long press (>2s): Reset counter\n");
    Serial.println("Press the UP button...\n");
}

void loop() {
    // Process pending interrupts (important for reliability)
    mcp.processPendingInterrupts();

    // Handle button events
    if (buttonEvent) {
        buttonEvent = false;

        // Toggle LED
        ledState = !ledState;
        if (ledState) {
            led.high();  // LED ON (active-low, so GPIO goes LOW)
        } else {
            led.low();   // LED OFF (active-low, so GPIO goes HIGH)
        }

        // Increment counter
        toggleCount++;

        // Print status
        Serial.print("Button pressed! LED: ");
        Serial.print(ledState ? "ON " : "OFF");
        Serial.print(" | Count: ");
        Serial.println(toggleCount);

        // Check if LED state matches what we set
        if (mcp.getLastError() != MCP23009_OK) {
            Serial.println("⚠ Error communicating with MCP23009E!");
        }
    }

    // Show status every 5 seconds
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        lastStatusTime = millis();
        Serial.println("\n--- Status ---");
        Serial.print("LED state: ");
        Serial.println(ledState ? "ON" : "OFF");
        Serial.print("Toggle count: ");
        Serial.println(toggleCount);
        Serial.println("--------------\n");
    }

    delay(10);
}
