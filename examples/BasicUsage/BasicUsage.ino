/**
 * Basic Usage Example for MCP23009E Library
 *
 * This example demonstrates basic GPIO configuration and reading/writing.
 * It reads the D-PAD buttons and prints their state to the Serial monitor.
 *
 * Hardware Setup:
 * - MCP23009E connected via I2C
 * - Reset pin connected to RST_EXPANDER
 * - D-PAD buttons on GPIO 4, 5, 6, 7
 *
 * D-PAD Button Mapping:
 * - GPIO 7: UP
 * - GPIO 5: DOWN
 * - GPIO 6: LEFT
 * - GPIO 4: RIGHT
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Reset pin (adjust for your board)
const int RESET_PIN = -1;  // Set to your reset pin number, or -1 if none

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // Wait for Serial to be ready

    Serial.println("================================");
    Serial.println("MCP23009E Basic Usage Example");
    Serial.println("================================\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN);

    // Configure D-PAD buttons as inputs with pull-ups
    Serial.println("Configuring D-PAD buttons...");
    mcp.setup(MCP23009_BTN_UP, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_DOWN, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_LEFT, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    mcp.setup(MCP23009_BTN_RIGHT, MCP23009_DIR_INPUT, MCP23009_PULLUP);
    Serial.println("✓ Configuration complete\n");

    // Read initial state
    Serial.println("Initial button states:");
    Serial.print("  UP:    "); Serial.println(mcp.getLevel(MCP23009_BTN_UP) ? "HIGH" : "LOW");
    Serial.print("  DOWN:  "); Serial.println(mcp.getLevel(MCP23009_BTN_DOWN) ? "HIGH" : "LOW");
    Serial.print("  LEFT:  "); Serial.println(mcp.getLevel(MCP23009_BTN_LEFT) ? "HIGH" : "LOW");
    Serial.print("  RIGHT: "); Serial.println(mcp.getLevel(MCP23009_BTN_RIGHT) ? "HIGH" : "LOW");
    Serial.println("\nPress buttons (LOW = pressed, HIGH = released)...\n");
}

// Store previous button states
uint8_t lastUp = HIGH;
uint8_t lastDown = HIGH;
uint8_t lastLeft = HIGH;
uint8_t lastRight = HIGH;

void loop() {
    // Read current button states
    uint8_t up = mcp.getLevel(MCP23009_BTN_UP);
    uint8_t down = mcp.getLevel(MCP23009_BTN_DOWN);
    uint8_t left = mcp.getLevel(MCP23009_BTN_LEFT);
    uint8_t right = mcp.getLevel(MCP23009_BTN_RIGHT);

    // Detect and print changes
    if (up != lastUp) {
        Serial.print("UP: ");
        Serial.println(up ? "RELEASED" : "PRESSED");
        lastUp = up;
    }

    if (down != lastDown) {
        Serial.print("DOWN: ");
        Serial.println(down ? "RELEASED" : "PRESSED");
        lastDown = down;
    }

    if (left != lastLeft) {
        Serial.print("LEFT: ");
        Serial.println(left ? "RELEASED" : "PRESSED");
        lastLeft = left;
    }

    if (right != lastRight) {
        Serial.print("RIGHT: ");
        Serial.println(right ? "RELEASED" : "PRESSED");
        lastRight = right;
    }

    delay(50);  // Small delay to avoid excessive I2C traffic
}
