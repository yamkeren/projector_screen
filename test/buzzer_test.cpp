// ============================================================================
// test/buzzer_test.cpp — Sequential buzzer sound demo
//
// Plays each buzzer sound in order with a Serial label, waits 2 seconds
// between sounds, then loops forever.
//
// Build & upload:
//   pio run -e buzzer_test -t upload
//   pio device monitor -e buzzer_test
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "audio/buzzer.h"

static Buzzer buzzer;

// ---------------------------------------------------------------------------
// Helper: play a sound, pump update() until it finishes, then wait 2 s
// ---------------------------------------------------------------------------
static void playAndWait(const char* label, void (Buzzer::*fn)()) {
    Serial.printf("\n>> %s\n", label);

    (buzzer.*fn)();                             // enqueue the sound

    // Pump the playback FSM until the pattern completes
    while (true) {
        buzzer.update();
        if (!buzzer.isPlaying()) break;
        delay(5);                               // ~5 ms tick, fine for a test
    }

    Serial.println("   Done.\n   Waiting 2 seconds...");
    delay(2000);
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  Buzzer Sound Test");
    Serial.printf("  GPIO %d  |  LEDC ch %d\n",
                  cfg::buzzer::PIN, cfg::buzzer::PWM_CHANNEL);
    Serial.println("========================================");

    buzzer.begin();
    delay(500);
}

// ---------------------------------------------------------------------------
void loop() {
    Serial.println("\n--- Starting sound cycle ---\n");

    playAndWait("1/6  WiFi Connecting",    &Buzzer::playWifiConnecting);
    playAndWait("2/6  WiFi Success",       &Buzzer::playWifiSuccess);
    playAndWait("3/6  WiFi Failure",       &Buzzer::playWifiFailure);
    playAndWait("4/6  Homing",             &Buzzer::playHoming);
    playAndWait("5/6  Screen Down (Open)", &Buzzer::playScreenDown);
    playAndWait("6/6  Screen Closing",     &Buzzer::playScreenClosing);

    Serial.println("\n=== Cycle complete — restarting in 3 seconds ===\n");
    delay(3000);
}
