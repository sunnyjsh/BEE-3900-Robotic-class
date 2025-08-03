#include <Wire.h>

#define BUZZER_PIN A1

// ====== Sound Melodies & Tempos ======
const int marioMelody[] = { 659, 659, 0, 659, 0, 523, 659, 0, 784, 0 };
const int marioTempo[] = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };
const int marioLength = sizeof(marioMelody) / sizeof(int);

const int twinkleMelody[] = { 262, 262, 392, 392, 440, 440, 392, 0, 349, 349, 330, 330, 294, 294, 262, 0 };
const int twinkleTempo[] = { 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 2, 4 };
const int twinkleLength = sizeof(twinkleMelody) / sizeof(int);

const int failMelody[] = { 196, 185, 175, 165 };
const int failTempo[] = { 8, 8, 8, 4 };
const int failLength = sizeof(failMelody) / sizeof(int);

const int wompMelody[] = { 196, 131 };
const int wompTempo[] = { 4, 2 };
const int wompLength = sizeof(wompMelody) / sizeof(int);

void setup() {
  // put your setup code here, to run once:
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  playTune(wompMelody, wompTempo, wompLength);
  delay(3000);
  playTune(marioMelody, marioTempo, marioLength);
  delay(3000);
  playTune(twinkleMelody, twinkleTempo, twinkleLength);
  delay(3000);
  playTune(failMelody, failTempo, failLength);
  delay(3000);
}

/**
 * Generic function to play a tune on the buzzer.
 */
void playTune(const int melody[], const int tempo[], int length) {
  for (int i = 0; i < length; i++) {
    int note = melody[i];
    int duration = 1000 / tempo[i];
    if (note == 0) {  // rest note
      delay(duration);
    } else {
      tone(BUZZER_PIN, note, duration);
      delay(duration * 1.3);  // Brief pause between notes
    }
  }
  noTone(BUZZER_PIN);  // Stop sound
}
