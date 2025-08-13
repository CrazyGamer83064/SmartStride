/**
 * TF-Luna LiDAR Sensor with Strobe Light Activation (Sensor Pauses during Strobe)
 *
 * This sketch reads distance data from the Benewake TF-Luna LiDAR sensor.
 * When an object comes within 50 cm of the sensor, strobe lights (connected via MOSFET)
 * will activate for 5 seconds with a 100 millisecond interval between flashes.
 *
 * CRUCIALLY: The LiDAR sensor will pause its detection/reading process while the
 * strobe lights are active to prevent interference and resume when they deactivate.
 *
 * Designed for Arduino Uno R3.
 *
 * HARDWARE SETUP:
 * 1.  TF-Luna LiDAR Sensor (I2C Mode):
 * - TF-Luna VCC -> Arduino 5V
 * - TF-Luna GND -> Arduino GND
 * - TF-Luna SDA -> Arduino A4 (SDA)
 * - TF-Luna SCL -> Arduino A5 (SCL)
 * - TF-Luna CFG -> Arduino GND (CRITICAL for I2C mode)
 * - TF-Luna MUX -> (Unconnected)
 *
 * 2.  Strobe Lights (connected via MOSFET):
 * - MOSFET Gate pin -> Arduino Digital Pin 9
 * - MOSFET Source pin -> Arduino GND
 * - MOSFET Drain pin -> Negative (short) leg of the LED/Strobe lights
 * - Positive (long) leg of the LED/Strobe lights -> Current-limiting resistor (e.g., 220-470 ohm, or appropriate for your lights)
 * - Other end of resistor -> 5V (or external power supply's positive terminal for higher power)
 * - IMPORTANT: If using an external power supply for the lights, ensure its GND is connected to Arduino GND.
 *
 * LIBRARIES NEEDED:
 * 1.  TFLI2C Library (Download via Arduino IDE Library Manager)
 * 2.  Wire Library (Usually built-in with Arduino IDE)
 */

#include "Wire.h"    // Required for I2C communication
#include "TFLI2C.h"  // Library for TF-Luna I2C communication

// TF-Luna Sensor Configuration
TFLI2C sensor; // Create an instance of the TFLI2C sensor object
const int TRIGGER_DISTANCE_CM = 180; // Distance threshold in centimeters

// Strobe Light Configuration
const int ledPin = 9;         // Digital pin connected to the MOSFET's Gate
const long STROBE_ACTIVE_DURATION = 20000; // Strobe active duration in milliseconds (5 seconds)
const int FLASH_INTERVAL_MS = 400; // Interval between LED on/off cycles for strobing (100 milliseconds)

// Non-blocking timing variables for strobe
unsigned long strobeStartTime = 0; // When the strobe effect started
unsigned long lastFlashToggleTime = 0; // When the LED last changed state
bool strobeActive = false; // Flag to indicate if strobe is currently running
bool ledState = LOW; // Current state of the LED (LOW for off, HIGH for on)

// To control sensor reading frequency (optional, but good practice if not strobing)
unsigned long lastSensorReadTime = 0;
const long SENSOR_READ_INTERVAL_MS = 100; // Read sensor every 100ms when strobe is off


void setup() {
  // Initialize serial communication for the Serial Monitor.
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to connect

  Serial.println("Arduino TF-Luna Strobe Project Started");

  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure strobe is off initially

  // Initialize the I2C bus for the TF-Luna.
  Wire.begin();
  Serial.println("I2C bus initialized.");

  // Give the sensor a brief moment to stabilize.
  delay(100);

  Serial.println("Monitoring TF-Luna sensor for objects within " + String(TRIGGER_DISTANCE_CM) + " cm...");
}


void loop() {
  unsigned long currentMillis = millis(); // Get current time for non-blocking operations

  // --- Strobe Light Control (Non-Blocking, takes priority) ---
  if (strobeActive) {
    // If strobe is active, first check if its active duration is over
    if (currentMillis - strobeStartTime >= STROBE_ACTIVE_DURATION) {
      strobeActive = false;
      digitalWrite(ledPin, LOW); // Ensure LED is off
      Serial.println("Strobe completed its 5-second cycle. Resuming sensor readings.");
    } else {
      // Strobe is still active, continue flashing
      if (currentMillis - lastFlashToggleTime >= FLASH_INTERVAL_MS) {
        lastFlashToggleTime = currentMillis; // Update last toggle time
        ledState = !ledState; // Toggle LED state
        digitalWrite(ledPin, ledState);
        // Serial.println("DEBUG: Toggling LED to " + String(ledState == HIGH ? "HIGH" : "LOW")); // Optional debug print
      }
      // Since strobe is active, we don't proceed to sensor reading in this loop iteration
      return; // Exit loop() early, will re-enter on next pass
    }
  }

  // --- TF-Luna Sensor Reading (Only when strobe is NOT active) ---
  // We'll read the sensor periodically using millis() to avoid blocking
  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadTime = currentMillis; // Update the last read time

    int16_t dist; // Variable to store the distance measurement

    if (sensor.getData(dist, 0x10)) {
      // If data is valid, print the distance.
      Serial.print("Distance: ");
      Serial.print(dist); // Distance is typically in centimeters (cm)
      Serial.println(" cm");

      // --- Strobe Activation Logic ---
      if (dist <= TRIGGER_DISTANCE_CM && dist > 0) { // Check if object is within range (and not an error)
        if (!strobeActive) { // If strobe is not already active, activate it
          strobeActive = true;
          strobeStartTime = currentMillis; // Record when the strobe started
          digitalWrite(ledPin, HIGH); // Turn on LED immediately for first flash
          ledState = HIGH; // Set initial state
          lastFlashToggleTime = currentMillis; // Initialize flash timer
          Serial.println("Object detected! Activating strobe. Sensor paused.");
        }
      } else {
        // If strobe was active and object moved out of range before full duration,
        // this part will be handled by the initial `if (strobeActive)` block's duration check.
        // We only need to worry about activating it here.
      }

    } else {
      // If sensor reading fails, print the error.
      Serial.print("Sensor Error: ");
      sensor.printStatus();
      Serial.println();
      // Even if sensor errors, it will continue to attempt readings when strobe is off.
    }
  }
}