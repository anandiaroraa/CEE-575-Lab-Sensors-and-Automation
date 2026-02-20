#include "Particle.h"
#include <math.h>

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Pins to RGB LED 
int pinR = D0;
int pinG = D1;
// Use D2 for blue channel
int pinB = D2;


// Helper to write RGB values while handling common-anode inversion
void writeRGB(int r, int g, int b) {
  if (commonAnode) {
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  }
  // If value is full-on or full-off, use digitalWrite for reliability
  auto writeChannel = [](int pin, int value) {
    if (value <= 0) {
      digitalWrite(pin, LOW);
    } else if (value >= 255) {
      digitalWrite(pin, HIGH);
    } else {
      analogWrite(pin, value);
    }
  };

  writeChannel(pinR, r);
  writeChannel(pinG, g);
  writeChannel(pinB, b);
}

// Current operating mode (1..4). Default to Mode 1 (All ON)
volatile int currentMode = 1;
// Track last mode to detect changes
int lastMode = -1;

// Forward declarations for mode functions
void modeAllOn();
void modeSmoothCycle();
void modeAlertFlash();
void modePartyMode();
// Helper utilities
void hsvToRgb(float h, float s, float v, int &outR, int &outG, int &outB);
bool checkSerialForModeChange();


int setMode(String cmd);


void enterMode(int mode);


void setup() {
  // Configure pins as outputs
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);

  Serial.begin(115200);
  while(!Serial && millis() < 2000) {
    // wait briefly for Serial to become available on USB
  }


  Particle.function("mode", setMode);

  Log.info("Lamp controller initialized. Send '1'..'4' over Serial to change modes or call Particle function 'mode'.");
}


void loop() {
  // Non-blocking check for serial input to change mode
  checkSerialForModeChange();

  // If mode changed, log and let per-mode init run on next loop
  if (currentMode != lastMode) {
    Log.info("Mode change: %d -> %d", lastMode, currentMode);
    lastMode = currentMode;
  }

  // Call the non-blocking mode update for the active mode
  switch (currentMode) {
    case 1: modeAllOn(); break;
    case 2: modeSmoothCycle(); break;
    case 3: modeAlertFlash(); break;
    case 4: modePartyMode(); break;
    default: currentMode = 1; break;
  }
}

//Modes

// Mode 1: All lights ON 

void modeAllOn() {
  static bool init = false;
  if (!init) {
    Log.info("Mode 1: All lights ON (static white)");
    init = true;
  }

  writeRGB(255,255,255);


  if (currentMode != 1) init = false;
}

// Mode 2: Smooth color cycle 
void modeSmoothCycle() {
  static bool init = false;
  static float hue = 0.0f;
  static const float hueStep = 1.0f;
  static const unsigned long stepDelay = 20;
  static unsigned long last = 0;

  if (!init) {
    Log.info("Mode 2: Smooth color cycle");
    hue = 0.0f;
    last = millis();
    init = true;
  }

  unsigned long now = millis();
  if (now - last >= stepDelay) {
    last = now;
    int r,g,b;
    hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
    writeRGB(r,g,b);
    hue += hueStep;
    if (hue >= 360.0f) hue -= 360.0f;
  }

  if (currentMode != 2) init = false;
}

// Mode 3: Alert flash
void modeAlertFlash() {
  static bool init = false;
  static const unsigned long onTime = 120;
  static const unsigned long offTime = 120;
  static unsigned long nextToggle = 0;
  static bool stateOn = false;

  if (!init) {
    Log.info("Mode 3: Alert flash");
    stateOn = false;
    nextToggle = millis();
    init = true;
  }

  unsigned long now = millis();
  if (now >= nextToggle) {
    stateOn = !stateOn;
    if (stateOn) {
      writeRGB(255,255,255);
      nextToggle = now + onTime;
    } else {
      writeRGB(0,0,0);
      nextToggle = now + offTime;
    }
  }

  if (currentMode != 3) init = false;
}

// Mode 4: Party mode: random colors on the beats
// Song used for beat reference: //https://www.youtube.com/watch?v=3AtDnEC4zak
void modePartyMode() {
  static bool init = false;
  static const float bpm = 120.0f;
  static const unsigned long beatMs = (unsigned long)(60000.0f / bpm);
  static unsigned long beatStart = 0;
  static int r0=0,g0=0,b0=0;
  static int r1=0,g1=0,b1=0;

  if (!init) {
    Log.info("Mode 4: Party mode (random colors on beat)");
    r0 = random(0,256); g0 = random(0,256); b0 = random(0,256);
    r1 = random(0,256); g1 = random(0,256); b1 = random(0,256);
    beatStart = millis();
    init = true;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - beatStart;
  if (elapsed >= beatMs) {
    // shift colors and pick new target
    r0 = r1; g0 = g1; b0 = b1;
    r1 = random(0,256); g1 = random(0,256); b1 = random(0,256);
    beatStart = now;
    elapsed = 0;
  }

  float t = (float)elapsed / (float)beatMs;
  int rc = (int)((1.0f - t) * r0 + t * r1);
  int gc = (int)((1.0f - t) * g0 + t * g1);
  int bc = (int)((1.0f - t) * b0 + t * b1);
  writeRGB(rc,gc,bc);

  if (currentMode != 4) init = false;
}

int setMode(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return -1;
  char c = cmd.charAt(0);
  if (c >= '1' && c <= '4') {
    int newMode = c - '0';
    if (newMode != currentMode) {
      Log.info("[func] Switching to mode %d", newMode);
      currentMode = newMode;
      // perform any immediate actions for the newly selected mode
      enterMode(newMode);
    }
    return currentMode;
  }
  return -1;
}
bool checkSerialForModeChange() {
  bool changed = false;
  while (Serial.available()) {
    int b = Serial.read();
    if (b <= 0) continue;
    char c = (char)b;
    if (c == '\r' || c == '\n') continue;
    if (c >= '1' && c <= '4') {
      int newMode = c - '0';
      if (newMode != currentMode) {
        Log.info("[serial] Switching to mode %d", newMode);
        currentMode = newMode;
        // apply mode immediately
        enterMode(newMode);
        changed = true;
      }
    }
  }
  return changed;
}

// Immediate actions when entering a mode. Keeps Mode 1 instant.
void enterMode(int mode) {
  if (mode == 1) {
    // Force all channels fully ON immediately (explicitly set pin levels)
    Log.info("[enterMode] Forcing Mode 1 outputs (commonAnode=%d)", commonAnode);
    // Ensure pins are outputs
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
    if (commonAnode) {
      // common-anode: drive LOW to turn LED on
      digitalWrite(pinR, LOW);
      digitalWrite(pinG, LOW);
      digitalWrite(pinB, LOW);
    } else {
      // common-cathode: drive HIGH to turn LED on
      digitalWrite(pinR, HIGH);
      digitalWrite(pinG, HIGH);
      digitalWrite(pinB, HIGH);
    }
    // Also set PWM/full value for boards that use analogWrite
    writeRGB(255,255,255);
  } else {
    // For other modes we don't need immediate action here.
  }
}

// Convert HSV (h in degrees 0..360, s,v in 0..1) to RGB 0..255
void hsvToRgb(float h, float s, float v, int &outR, int &outG, int &outB) {
  float c = v * s;
  float hh = fmodf(h / 60.0f, 6.0f);
  float x = c * (1 - fabsf(fmodf(hh,2.0f) - 1));
  float m = v - c;
  float r=0,g=0,b=0;

  if (0 <= hh && hh < 1) { r = c; g = x; b = 0; }
  else if (1 <= hh && hh < 2) { r = x; g = c; b = 0; }
  else if (2 <= hh && hh < 3) { r = 0; g = c; b = x; }
  else if (3 <= hh && hh < 4) { r = 0; g = x; b = c; }
  else if (4 <= hh && hh < 5) { r = x; g = 0; b = c; }
  else if (5 <= hh && hh < 6) { r = c; g = 0; b = x; }

  outR = (int)((r + m) * 255.0f);
  outG = (int)((g + m) * 255.0f);
  outB = (int)((b + m) * 255.0f);
}
