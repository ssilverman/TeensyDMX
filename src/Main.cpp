#include "TeensyDMX.h"

// Necessary for processing the full DMX packet size.
#define SERIAL1_RX_BUFFER_SIZE 513

#define CHANNEL        1
#define DMX_TIMEOUT    1000
#define LED_PIN        13
#define PRINT_INTERVAL 1000

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::TeensyDMX dmx{Serial1};

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTime;

// The last value sent to CHANNEL.
uint8_t lastValue;

void setup() {
  Serial.begin(9600);
  delay(2000);  // Instead of while (!Serial), doesn't seem to work on Teensy
  Serial.println("Starting.");

  pinMode(LED_PIN, OUTPUT);

  dmx.begin();
  lastFrameTime = DMX_TIMEOUT;
}

void loop() {
  static uint8_t buf[teensydmx::TeensyDMX::kMaxDMXPacketSize];
  int read = dmx.readPacket(buf);
  if (read > CHANNEL) {
    lastValue = buf[CHANNEL];

    // Print the data ever so often
    static elapsedMillis p = PRINT_INTERVAL;
    if (p >= PRINT_INTERVAL) {
      Serial.printf("(%d)", read);
      for (int i = 0; i < 10; i++) {
        Serial.printf(" %d:%d", i, buf[CHANNEL + i]);
      }
      Serial.println();
      p = 0;
    }
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
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
