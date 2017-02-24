#include "TeensyDMX.h"

// Necessary for processing the full DMX packet size.
#define SERIAL1_RX_BUFFER_SIZE 513

#define CHANNEL        1
#define DMX_TIMEOUT    1000
#define LED_PIN        13
#define PRINT_INTERVAL 2000

// Create the DMX receiver on Serial1.
::qindesign::teensydmx::TeensyDMX dmx{Serial1};

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTime;

// The last value sent to CHANNEL.
uint8_t lastValue;

// Keeps track of the last time we printed a value to the serial monitor.
elapsedMillis lastPrintTime;

void setup() {
  Serial.begin(9600);
  delay(2000);  // Instead of while (!Serial), doesn't seem to work on Teensy
  Serial.println("Starting.");

  pinMode(LED_PIN, OUTPUT);

  dmx.begin();
  lastFrameTime = DMX_TIMEOUT;
  lastPrintTime = 0;
}

void loop() {
  static uint8_t buf[teensydmx::TeensyDMX::kMaxDMXPacketSize];
  int read = dmx.readPacket(buf);
  if (read >= 0) {
    lastValue = buf[CHANNEL];
    lastFrameTime = 0;
  }

  if (lastFrameTime < DMX_TIMEOUT) {
    // Use a wave equation to make the speed-ups and slow-downs smoother
    // using the offset, phi
    static int period = 1000;
    static long phi = 0 * period;
    int newPeriod = map(lastValue, 0, 255, 1000, 30);  // T range is 1s to 30ms
    long t = static_cast<long>(millis());
    phi = (t*period - newPeriod*(t - phi))/period;
    period = newPeriod;
    int v = static_cast<int>(-(phi - t)%period);

    if (v < period/2) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    if (lastPrintTime >= PRINT_INTERVAL) {
      Serial.println(lastValue);
      lastPrintTime = 0;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
