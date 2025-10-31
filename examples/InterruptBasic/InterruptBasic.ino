/**
 * Basic Interrupt Example for MCP23009E Library
 *
 * This example demonstrates how to use hardware interrupts with the MCP23009E.
 * When a button is pressed, an interrupt is triggered and handled automatically.
 *
 * Hardware Setup:
 * - MCP23009E connected via I2C
 * - Button on GPIO 7 (with pull-up enabled)
 * - MCP23009E INT pin connected to microcontroller GPIO (INT_EXPANDER)
 *
 * Interrupt modes:
 * - FALLING: Triggered when button is pressed (HIGH to LOW)
 * - RISING: Triggered when button is released (LOW to HIGH)
 * - CHANGE: Triggered on both press and release
 *
 * Note: The interrupt pin configuration and ISR attachment are now handled
 *       automatically by the library when you pass the interruptPin to begin().
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Reset pin (adjust for your board)
const int RESET_PIN = RST_EXPANDER;  // Set to your reset pin number, or -1 if none
const int INTERRUPT_PIN = INT_EXPANDER;  // Microcontroller pin connected to MCP23009E INT

// Button configuration
const int BUTTON_PIN = MCP23009_BTN_UP;  // Use D-PAD UP button (GPIO 7)

// Volatile variables for interrupt handling
volatile bool buttonPressed = false;
volatile bool buttonReleased = false;

// Callback for button press (falling edge)
void onButtonPress() {
    buttonPressed = true;
}

// Callback for button release (rising edge)
void onButtonRelease() {
    buttonReleased = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Initializing I2C...");
    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();

    Serial.println("================================");
    Serial.println("MCP23009E Interrupt Basic Example");
    Serial.println("================================\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN, INTERRUPT_PIN);

    // Configure button as input with pull-up
    Serial.println("Configuring button with interrupts...");
    mcp.setup(BUTTON_PIN, MCP23009_DIR_INPUT, MCP23009_PULLUP);

    // Configure interrupts on the MCP23009E
    // Note: The interrupt pin and ISR are automatically configured by begin()
    mcp.interruptOnFalling(BUTTON_PIN, onButtonPress);
    mcp.interruptOnRaising(BUTTON_PIN, onButtonRelease);

    Serial.println("✓ Interrupt configuration complete");
    Serial.println("\nPress the UP button to trigger interrupts...\n");
    Serial.println("(Button should be connected with pull-up)");
    Serial.println("Press = FALLING interrupt");
    Serial.println("Release = RISING interrupt\n");
}

uint32_t pressCount = 0;
uint32_t releaseCount = 0;

void loop() {
    // Handle button press events in main loop (not in ISR)
    if (buttonPressed) {
        buttonPressed = false;
        pressCount++;
        Serial.print("✓ Button PRESSED (count: ");
        Serial.print(pressCount);
        Serial.println(")");
    }

    // Handle button release events in main loop (not in ISR)
    if (buttonReleased) {
        buttonReleased = false;
        releaseCount++;
        Serial.print("✓ Button RELEASED (count: ");
        Serial.print(releaseCount);
        Serial.println(")");
    }

    // Print status every 5 seconds
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        lastStatusTime = millis();
        Serial.println("\n--- Status ---");
        Serial.print("Total presses: ");
        Serial.println(pressCount);
        Serial.print("Total releases: ");
        Serial.println(releaseCount);
        Serial.println("Waiting for interrupts...\n");
    }

    // Small delay to avoid excessive CPU usage
    delay(10);
}
