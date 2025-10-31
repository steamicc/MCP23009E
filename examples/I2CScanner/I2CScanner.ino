/**
 * I2C Scanner for STEAMI Board
 *
 * This simple sketch scans the I2C bus to find all connected devices.
 * Use this to verify that your MCP23009E is connected and responding.
 */

#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n========================================");
    Serial.println("I2C Scanner for STEAMI");
    Serial.println("========================================\n");

    // Configure MCP23009E reset pin BEFORE I2C initialization
    Serial.println("Configuring MCP23009E reset...");
    pinMode(RST_EXPANDER, OUTPUT);
    digitalWrite(RST_EXPANDER, HIGH);  // Keep reset inactive
    Serial.println("✓ Reset pin configured\n");

    // Initialize I2C with STEAMI pins
    Serial.println("Initializing I2C...");
    Serial.print("  SDA: ");
    Serial.println(I2C_INT_SDA);
    Serial.print("  SCL: ");
    Serial.println(I2C_INT_SCL);

    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();

    Serial.println("✓ I2C initialized\n");

    // Perform hardware reset on MCP23009E
    Serial.println("Resetting MCP23009E...");
    digitalWrite(RST_EXPANDER, LOW);
    delay(5);
    digitalWrite(RST_EXPANDER, HIGH);
    delay(10);
    Serial.println("✓ MCP23009E reset complete\n");

    Serial.println("Scanning I2C bus...\n");

    uint8_t devicesFound = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Device found at address 0x");
            if (address < 0x10) Serial.print("0");
            Serial.print(address, HEX);
            Serial.print(" (");
            Serial.print(address, DEC);
            Serial.print(")");

            // Identify known devices
            if (address == 0x20) {
                Serial.print(" - Likely MCP23009E");
            }

            Serial.println();
            devicesFound++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 0x10) Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    Serial.println("\n========================================");
    if (devicesFound == 0) {
        Serial.println("⚠ No I2C devices found!");
        Serial.println("\nPossible causes:");
        Serial.println("  - No devices connected");
        Serial.println("  - Wrong I2C pins configured");
        Serial.println("  - Missing pull-up resistors");
        Serial.println("  - Device not powered");
    } else {
        Serial.print("✓ Found ");
        Serial.print(devicesFound);
        Serial.println(" device(s)");
    }
    Serial.println("========================================\n");
}

void loop() {
    // Rescan every 5 seconds
    delay(5000);

    Serial.println("Rescanning...");
    uint8_t devicesFound = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            devicesFound++;
        }
    }

    Serial.print("Devices found: ");
    Serial.println(devicesFound);
}
