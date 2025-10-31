/**
 * LED Active-Low Example for MCP23009E Library
 *
 * This example demonstrates controlling LEDs in active-low configuration.
 * In this configuration, LEDs are connected between VCC and GPIO.
 *
 * Hardware Setup:
 * - Connect LEDs: 3.3V → [LED] → [Resistor 220-330Ω] → GPIO 0-3
 *
 * The MCP23009E can sink more current (25mA) than it can source (~1mA).
 * Therefore, this active-low configuration is recommended for LEDs.
 *
 * With MCP23009ActiveLowPin, you use normal logic:
 * - led.high() → turns LED ON
 * - led.low()  → turns LED OFF
 */

#include <Wire.h>
#include <MCP23009E.h>

// Create MCP23009E instance
MCP23009E mcp(Wire, MCP23009_I2C_ADDR);

// Create active-low LED pins
MCP23009ActiveLowPin led0(mcp, 0);
MCP23009ActiveLowPin led1(mcp, 1);
MCP23009ActiveLowPin led2(mcp, 2);
MCP23009ActiveLowPin led3(mcp, 3);

MCP23009ActiveLowPin* leds[] = {&led0, &led1, &led2, &led3};
const int NUM_LEDS = 4;

// Reset pin (adjust for your board)
const int RESET_PIN = RST_EXPANDER;  // Set to your reset pin number, or -1 if none

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Initializing I2C...");
    Wire.setSDA(I2C_INT_SDA);
    Wire.setSCL(I2C_INT_SCL);
    Wire.begin();

    Serial.println("================================");
    Serial.println("MCP23009E LED Active-Low Example");
    Serial.println("================================\n");
    Serial.println("Hardware setup:");
    Serial.println("  3.3V → [LED] → [R] → GPIO\n");

    // Initialize MCP23009E
    mcp.begin(RESET_PIN);

    // Configure all LED pins as outputs after I2C initialization
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i]->pinMode(OUTPUT);
        leds[i]->low();  // Start with all LEDs off
    }

    Serial.println("✓ LEDs configured (all OFF)\n");

    // Test 1: Sequential blink
    Serial.println("Test 1: Sequential blink");
    for (int i = 0; i < NUM_LEDS; i++) {
        Serial.print("  LED "); Serial.print(i); Serial.println(": ON");
        leds[i]->high();
        delay(300);
        Serial.print("  LED "); Serial.print(i); Serial.println(": OFF");
        leds[i]->low();
        delay(200);
    }

    Serial.println("\nTest 2: Simultaneous blink");
    for (int j = 0; j < 3; j++) {
        Serial.println("  All LEDs: ON");
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i]->high();
        }
        delay(300);

        Serial.println("  All LEDs: OFF");
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i]->low();
        }
        delay(300);
    }

    Serial.println("\nTest 3: Knight Rider effect (3 cycles)");
    for (int cycle = 0; cycle < 3; cycle++) {
        // Forward
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i]->high();
            delay(100);
            leds[i]->low();
        }
        // Backward
        for (int i = NUM_LEDS - 1; i >= 0; i--) {
            leds[i]->high();
            delay(100);
            leds[i]->low();
        }
    }

    Serial.println("\nSetup complete. Starting continuous pattern...\n");
}

void loop() {
    // Binary counter on 4 LEDs
    static uint8_t count = 0;

    Serial.print("Count: "); Serial.print(count);
    Serial.print(" = ");
    for (int i = 3; i >= 0; i--) {
        Serial.print((count >> i) & 1);
    }
    Serial.println();

    // Display count on LEDs (bit 0 = LED0, bit 3 = LED3)
    for (int i = 0; i < NUM_LEDS; i++) {
        if (count & (1 << i)) {
            leds[i]->high();
        } else {
            leds[i]->low();
        }
    }

    count++;
    if (count >= 16) count = 0;

    delay(500);
}
