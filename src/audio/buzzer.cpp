// ============================================================================
// buzzer.cpp — Sound pattern table for Buzzer
//
// Each pattern is an array of {freqHz, durationMs} steps.
// Frequencies are standard musical notes for recognizable, distinct sounds.
// A frequency of 0 represents a silence (rest) between tones.
//
// Pattern index mapping (SoundId enum value − 1):
//   [0] WIFI_CONNECTING  — rapid triple pulse: "connecting…"
//   [1] WIFI_SUCCESS      — ascending major 3rd: cheerful confirmation
//   [2] WIFI_FAILURE      — descending minor 3rd: sad failure
//   [3] HOMING            — two chirps + low tone: mechanical seek
//   [4] SCREEN_DOWN       — descending 3-step sweep: downward motion
//   [5] SCREEN_CLOSING    — ascending 3-step sweep: upward motion
// ============================================================================

#include "audio/buzzer.h"

// All frequencies are standard musical notes in the 500-1400 Hz sweet spot
// for piezo speakers. Longer durations and wider gaps produce a warmer,
// more pleasant sound versus harsh short buzzes.
//
// Note reference:
//   C5=523  D5=587  E5=659  F5=698  G5=784  A5=880  B5=988
//   C6=1047 D6=1175 E6=1319  G4=392  A4=440  B4=494

const Buzzer::SoundPattern Buzzer::_patterns[] = {
    // -----------------------------------------------------------------------
    // [0] WIFI_CONNECTING — gentle double-tap at G5
    //     Musical feel: polite "please wait" knock
    // -----------------------------------------------------------------------
    { { {784, 100}, {0, 120}, {784, 100} }, 3 },

    // -----------------------------------------------------------------------
    // [1] WIFI_SUCCESS — ascending perfect 4th: C5 → E5 → G5
    //     Musical feel: bright major arpeggio — happy confirmation
    // -----------------------------------------------------------------------
    { { {523, 120}, {0, 40}, {659, 120}, {0, 40}, {784, 250} }, 5 },

    // -----------------------------------------------------------------------
    // [2] WIFI_FAILURE — descending minor 3rd: A4 → rest → F4 (long)
    //     Musical feel: "sad trombone" two-note — unmistakably negative
    // -----------------------------------------------------------------------
    { { {440, 300}, {0, 100}, {349, 500} }, 3 },

    // -----------------------------------------------------------------------
    // [3] HOMING — D5 staccato pair + resolved C5 hold
    //     Musical feel: precision "click-click-lock" for mechanical seek
    // -----------------------------------------------------------------------
    { { {587, 80}, {0, 100}, {587, 80}, {0, 100}, {523, 200} }, 5 },

    // -----------------------------------------------------------------------
    // [4] SCREEN_DOWN (open) — descending G5 → E5 → C5 (do-sol-mi glide)
    //     Musical feel: smooth falling scale — something lowering
    // -----------------------------------------------------------------------
    { { {784, 140}, {0, 50}, {659, 140}, {0, 50}, {523, 220} }, 5 },

    // -----------------------------------------------------------------------
    // [5] SCREEN_CLOSING (up) — ascending C5 → E5 → G5 (do-mi-sol rise)
    //     Musical feel: smooth rising scale — something retracting up
    // -----------------------------------------------------------------------
    { { {523, 140}, {0, 50}, {659, 140}, {0, 50}, {784, 220} }, 5 },
};
