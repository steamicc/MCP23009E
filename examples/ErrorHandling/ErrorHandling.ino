/**
 * Error Handling Example for MCP23009E Library
 *
 * This example demonstrates how to check for I2C communication errors
 * when working with the MCP23009E I/O expander.
 *
 * Hardware Setup:
 * - MCP23009E connected via I2C
 * - SDA, SCL properly connected with pull-up resistors
 * - Device powered correctly
 *
 * Error Codes:
 * - MCP23009_OK (0): Success
 * - MCP23009_ERROR_I2C_WRITE (1): I2C write failed
 * - MCP23009_ERROR_I2C_READ (2): I2C read failed
 * - MCP23009_ERROR_INVALID_PIN (3): Invalid pin number
 * - MCP23009_ERROR_TIMEOUT (4): I2C timeout
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

const int RESET_PIN = -1;  // Set to your reset pin, or -1 if none

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("================================");
    Serial.println("MCP23009E Error Handling Example");
    Serial.println("================================\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN);

    // Check if device is connected
    Serial.println("1. Checking device connection...");
    if (mcp.isConnected()) {
        Serial.println("   ✓ MCP23009E found at address 0x" + String(MCP23009_I2C_ADDR, HEX));
    } else {
        Serial.println("   ✗ MCP23009E not found!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
        Serial.println("\nPossible causes:");
        Serial.println("   - Wrong I2C address");
        Serial.println("   - Wiring issue (SDA/SCL)");
        Serial.println("   - Missing pull-up resistors");
        Serial.println("   - Device not powered");
        while (1) delay(100);  // Halt
    }

    // Clear any previous errors
    mcp.clearError();

    // Configure GPIO 7 as input with pull-up
    Serial.println("\n2. Configuring GPIO 7 as input...");
    mcp.setup(7, MCP23009_DIR_INPUT, MCP23009_PULLUP);

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println("   ✓ Configuration successful");
    } else {
        Serial.println("   ✗ Configuration failed!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
    }

    // Configure GPIO 0 as output
    Serial.println("\n3. Configuring GPIO 0 as output...");
    mcp.setup(0, MCP23009_DIR_OUTPUT);

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println("   ✓ Configuration successful");
    } else {
        Serial.println("   ✗ Configuration failed!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
    }

    // Test write operation
    Serial.println("\n4. Testing write operation...");
    mcp.setLevel(0, MCP23009_LOGIC_HIGH);

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println("   ✓ Write successful");
    } else {
        Serial.println("   ✗ Write failed!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
    }

    // Test read operation
    Serial.println("\n5. Testing read operation...");
    uint8_t level = mcp.getLevel(7);

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println("   ✓ Read successful");
        Serial.println("   Value: " + String(level ? "HIGH" : "LOW"));
    } else {
        Serial.println("   ✗ Read failed!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
    }

    // Test register access
    Serial.println("\n6. Testing register access...");
    uint8_t iodir = mcp.getIODIR();

    if (mcp.getLastError() == MCP23009_OK) {
        Serial.println("   ✓ Register read successful");
        Serial.println("   IODIR value: 0b" + String(iodir, BIN));
    } else {
        Serial.println("   ✗ Register read failed!");
        Serial.println("   Error code: " + String(mcp.getLastError()));
    }

    Serial.println("\n================================");
    Serial.println("All tests completed!");
    Serial.println("================================\n");
    Serial.println("Loop will monitor GPIO 7 and report any errors...\n");
}

uint32_t lastCheck = 0;
uint32_t errorCount = 0;
uint32_t successCount = 0;

void loop() {
    // Read GPIO 7 every 500ms
    if (millis() - lastCheck >= 500) {
        lastCheck = millis();

        uint8_t level = mcp.getLevel(7);

        if (mcp.getLastError() == MCP23009_OK) {
            successCount++;
            Serial.print("✓ Read #" + String(successCount) + ": ");
            Serial.println(level ? "HIGH" : "LOW");

            // Toggle output on GPIO 0
            mcp.setLevel(0, successCount % 2);
        } else {
            errorCount++;
            Serial.println("✗ Error #" + String(errorCount) + ": " + String(mcp.getLastError()));

            // Check if device is still connected
            if (!mcp.isConnected()) {
                Serial.println("   Device disconnected! Please check wiring.");
            }
        }
    }
}
