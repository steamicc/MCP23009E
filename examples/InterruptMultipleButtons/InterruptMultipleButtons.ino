/**
 * Multiple Buttons Interrupt Example for MCP23009E Library
 *
 * This example demonstrates handling interrupts from multiple buttons (D-PAD).
 * Each button can trigger an interrupt and the handler identifies which button
 * caused the interrupt and its state (pressed/released).
 *
 * Hardware Setup:
 * - MCP23009E connected via I2C
 * - D-PAD buttons on GPIO 4, 5, 6, 7 (with pull-ups enabled)
 * - MCP23009E INT pin connected to microcontroller GPIO (INT_EXPANDER)
 *
 * D-PAD Button Mapping:
 * - GPIO 7: UP
 * - GPIO 5: DOWN
 * - GPIO 6: LEFT
 * - GPIO 4: RIGHT
 *
 * Note: The interrupt pin configuration and ISR attachment are now handled
 *       automatically by the library when you pass the interruptPin to begin().
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Reset pin (adjust for your board)
const int RESET_PIN = RST_EXPANDER;
const int INTERRUPT_PIN = INT_EXPANDER;

// Button states
struct ButtonState {
    bool changed;
    uint8_t level;
    uint32_t pressCount;
};

volatile ButtonState upButton = {false, HIGH, 0};
volatile ButtonState downButton = {false, HIGH, 0};
volatile ButtonState leftButton = {false, HIGH, 0};
volatile ButtonState rightButton = {false, HIGH, 0};

// Individual button callbacks using CHANGE mode
void onUpButtonChange(uint8_t level) {
    upButton.changed = true;
    upButton.level = level;
    if (level == LOW) upButton.pressCount++;
}

void onDownButtonChange(uint8_t level) {
    downButton.changed = true;
    downButton.level = level;
    if (level == LOW) downButton.pressCount++;
}

void onLeftButtonChange(uint8_t level) {
    leftButton.changed = true;
    leftButton.level = level;
    if (level == LOW) leftButton.pressCount++;
}

void onRightButtonChange(uint8_t level) {
    rightButton.changed = true;
    rightButton.level = level;
    if (level == LOW) rightButton.pressCount++;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Initializing I2C...");
    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();

    Serial.println("========================================");
    Serial.println("MCP23009E Multiple Buttons Interrupt Example");
    Serial.println("========================================\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN, INTERRUPT_PIN);

    // Configure all D-PAD buttons with pull-ups
    Serial.println("Configuring D-PAD buttons with interrupts...");
    mcp.setup(MCP23009_BTN_UP, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_DOWN, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_LEFT, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_RIGHT, MCP23009_DIR_INPUT, MCP23009_PULLUP);

    // Configure interrupts for all buttons (CHANGE mode with level callback)
    // Note: The interrupt pin and ISR are automatically configured by begin()
    mcp.interruptOnChange(MCP23009_BTN_UP, onUpButtonChange);
    mcp.interruptOnChange(MCP23009_BTN_DOWN, onDownButtonChange);
    mcp.interruptOnChange(MCP23009_BTN_LEFT, onLeftButtonChange);
    mcp.interruptOnChange(MCP23009_BTN_RIGHT, onRightButtonChange);

    Serial.println("✓ All interrupts configured");
    Serial.println("\nPress any D-PAD button to see interrupt handling...");
    Serial.println("(Buttons: UP, DOWN, LEFT, RIGHT)\n");
}

void printButtonEvent(const char* buttonName, volatile ButtonState& button) {
    if (button.changed) {
        Serial.print("► ");
        Serial.print(buttonName);
        Serial.print(": ");
        Serial.print(button.level ? "RELEASED" : "PRESSED");
        Serial.print(" (total presses: ");
        Serial.print(button.pressCount);
        Serial.println(")");
    }
}

void loop() {
    // Check and handle all button events
    bool anyChange = false;

    if (upButton.changed) {
        printButtonEvent("UP   ", upButton);
        upButton.changed = false;
        anyChange = true;
    }

    if (downButton.changed) {
        printButtonEvent("DOWN ", downButton);
        downButton.changed = false;
        anyChange = true;
    }

    if (leftButton.changed) {
        printButtonEvent("LEFT ", leftButton);
        leftButton.changed = false;
        anyChange = true;
    }

    if (rightButton.changed) {
        printButtonEvent("RIGHT", rightButton);
        rightButton.changed = false;
        anyChange = true;
    }

    // Add a blank line after processing all changes
    if (anyChange) {
        Serial.println();
    }

    // Print statistics every 10 seconds
    static uint32_t lastStatsTime = 0;
    if (millis() - lastStatsTime > 10000) {
        lastStatsTime = millis();
        Serial.println("--- Button Press Statistics ---");
        Serial.print("UP: ");
        Serial.print(upButton.pressCount);
        Serial.print(" | DOWN: ");
        Serial.print(downButton.pressCount);
        Serial.print(" | LEFT: ");
        Serial.print(leftButton.pressCount);
        Serial.print(" | RIGHT: ");
        Serial.println(rightButton.pressCount);
        Serial.println("-------------------------------\n");
    }

    // Small delay
    delay(10);
}
