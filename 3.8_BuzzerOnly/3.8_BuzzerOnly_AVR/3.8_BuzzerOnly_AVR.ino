// ATmega328 Code to send a specific sound pattern command to ESP32
// This sketch is ONLY responsible for sending frequency data to the ESP32.

// Variable to store the frequency command being sent
uint16_t beepFrequency = 0;

// === Custom Pattern Setup: Twinkle Twinkle Little Star  ===

// Define common note frequencies (Approximate values in Hz)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_SILENCE 0 // Frequency 0 means silence/stop tone
#define NOTE_GAP 50    // Standard duration for a brief silence between notes (ms)

// Define the melody sequence (Frequency in Hz)
// We now explicitly interleave musical notes with silence (NOTE_SILENCE).
const uint16_t patternFrequencies[] = {
  // Line 1: C C G G A A G
  NOTE_C4, NOTE_SILENCE, NOTE_C4, NOTE_SILENCE, NOTE_G4, NOTE_SILENCE, NOTE_G4, NOTE_SILENCE, 
  NOTE_A4, NOTE_SILENCE, NOTE_A4, NOTE_SILENCE, NOTE_G4, NOTE_SILENCE, NOTE_SILENCE, // Long G note and final pause
  
  // Line 2: F F E E D D C
  NOTE_F4, NOTE_SILENCE, NOTE_F4, NOTE_SILENCE, NOTE_E4, NOTE_SILENCE, NOTE_E4, NOTE_SILENCE, 
  NOTE_D4, NOTE_SILENCE, NOTE_D4, NOTE_SILENCE, NOTE_C4, NOTE_SILENCE, NOTE_SILENCE, // Long C note and final pause
  
  // Line 3: G G F F E E D
  NOTE_G4, NOTE_SILENCE, NOTE_G4, NOTE_SILENCE, NOTE_F4, NOTE_SILENCE, NOTE_F4, NOTE_SILENCE, 
  NOTE_E4, NOTE_SILENCE, NOTE_E4, NOTE_SILENCE, NOTE_D4, NOTE_SILENCE, NOTE_SILENCE  // Long D note and end of loop rest
}; 

// Duration each frequency plays in milliseconds (Quarter note = 300ms, Half note = 600ms)
// Note duration is now 250ms (for quarter) or 550ms (for half) + 50ms silence (NOTE_GAP)
const uint16_t patternDurations[] = {
  // Line 1: C C G G A A G
  250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 
  250, NOTE_GAP, 250, NOTE_GAP, 550, NOTE_GAP, 100, // The long G note (550) is followed by a short silence (NOTE_GAP) and then a longer rest (100)
  
  // Line 2: F F E E D D C
  250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 
  250, NOTE_GAP, 250, NOTE_GAP, 550, NOTE_GAP, 100, 
  
  // Line 3: G G F F E E D
  250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 250, NOTE_GAP, 
  250, NOTE_GAP, 250, NOTE_GAP, 550, NOTE_GAP, 1500  // Long rest at the end (1500ms)
};   

const uint8_t patternLength = sizeof(patternFrequencies) / sizeof(patternFrequencies[0]);
uint8_t patternStep = 0;
unsigned long lastPatternTime = 0;
// =============================


void setup() {
  // Initialize Serial communication link with ESP32
  Serial.begin(115200);  
  
  Serial.println("ATmega Pattern Sender Initialized (Musical Twinkle).");
  
  // Send the first command immediately after initialization
  beepFrequency = patternFrequencies[0];
  Serial.print("BEEP:");
  Serial.println(beepFrequency);
  lastPatternTime = millis();
}

void loop() {
  // --- Pattern Timing Logic ---
  
  // Check if the current pattern step duration has expired
  if (millis() - lastPatternTime >= patternDurations[patternStep]) {
    lastPatternTime = millis();
    
    // Move to the next step
    patternStep++;
    
    // Check if the end of the pattern array is reached
    if (patternStep >= patternLength) {
      patternStep = 0;
      // Add a pause/silence before the pattern restarts
      delay(500); 
    }
    
    // Get the next frequency
    beepFrequency = patternFrequencies[patternStep];
    
    // Send the frequency command to the ESP32
    Serial.print("BEEP:");
    Serial.println(beepFrequency);
  }
}
