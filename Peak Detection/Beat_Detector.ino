// ===== SIMPLE CONTINUOUS PLOT VERSION =====
// Streams PPG + ECG + QRS + beat markers to Serial Plotter

// --------- Pin definitions ----------
const int PPG_PIN   = A0;          // Pulse sensor
const int ECG_PIN   = A1;          // AD8232 OUTPUT
const int LED_PIN   = LED_BUILTIN; // LED for PPG beats (optional)

// If you wired LO+ / LO-:
const int LO_PLUS   = 10;          // AD8232 LO+
const int LO_MINUS  = 11;          // AD8232 LO-

// --------- Sampling ----------
const unsigned long SAMPLE_INTERVAL_MS = 10;   // 100 Hz
unsigned long lastSampleTime  = 0;

// ---------------- PPG detection ----------------
const unsigned long PPG_REFRACTORY_MS = 250;   // avoid double PPG peaks

int   ppgThreshold = 620;      // tune in plotter
int   ppgBeatMarker = 0;

unsigned long ppgLastBeatTime = 0;
bool  ppgWasAboveThreshold    = false;

int   ppgMinSinceLastBeat = 1023;
int   ppgMaxSinceLastBeat = 0;

// ---------------- ECG + QRS detection ----------------
const int MAX_ECG_BEATS = 50;

float ecgRR[MAX_ECG_BEATS];    // not used now, but kept if you reuse later
int   ecgBeatCount = 0;

unsigned long ecgLastBeatTime = 0;
bool  ecgWasAboveThreshold    = false;

// QRS enhancement: derivative^2 + moving average
const int ENERGY_MA_LEN = 8;
long energyBuf[ENERGY_MA_LEN];
byte energyIndex = 0;
long energySum = 0;

int  prevEcgSample = 0;
int  qrsSignal = 0;

// Adaptive threshold on qrsSignal
int   qrsThreshold = 0;
bool  qrsThreshInit = false;
float qrsPeakAvg = 0.0;

const unsigned long ECG_REFRACTORY_MS = 400;   // robust up to ~150 bpm

int ecgBeatMarker = 0;          // 0 normally, 1023 at detected beat

// ===================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);

  Serial.begin(115200);
  // Give Serial Plotter a header line (names for each column):
  Serial.println("PPG PPGm ECG QRS ECGm");

  // init QRS state
  for (int i = 0; i < ENERGY_MA_LEN; i++) energyBuf[i] = 0;
  energySum = 0;
  energyIndex = 0;
  prevEcgSample = 0;
}

// ===================================================================

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime += SAMPLE_INTERVAL_MS;
    processSample(now);
  }
}

// ===================================================================
//                 SAMPLE PROCESSING (PPG + ECG)
// ===================================================================
void processSample(unsigned long now) {

  // default: no beat marks this sample
  ppgBeatMarker = 0;
  ecgBeatMarker = 0;

  // ----- Read PPG -----
  int ppg = analogRead(PPG_PIN);

  // Track min/max between beats for amplitude (for future use)
  if (ppg < ppgMinSinceLastBeat) ppgMinSinceLastBeat = ppg;
  if (ppg > ppgMaxSinceLastBeat) ppgMaxSinceLastBeat = ppg;

  // LED feedback for PPG
  digitalWrite(LED_PIN, (ppg > ppgThreshold));

  // ----- PPG peak detection -----
  if (ppg > ppgThreshold &&
      !ppgWasAboveThreshold &&
      (now - ppgLastBeatTime) > PPG_REFRACTORY_MS) {

    // (You could store IBI here if you want later)

    ppgLastBeatTime = now;
    ppgWasAboveThreshold = true;

    ppgMinSinceLastBeat = ppg;
    ppgMaxSinceLastBeat = ppg;

    // Mark this sample as PPG beat
    ppgBeatMarker = 1023;
  }
  else if (ppg < ppgThreshold) {
    ppgWasAboveThreshold = false;
  }

  // =====================  ECG (QRS-based)  =====================

  // ----- Read ECG -----
  int ecg = analogRead(ECG_PIN);

  // If leads off, force ECG to 0 (optional)
  if (digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH) {
    ecg = 0;
  }

  // ----- QRS enhancement: derivative -> square -> moving average -----
  int diff = ecg - prevEcgSample;
  prevEcgSample = ecg;

  long energy = (long)diff * (long)diff;

  energySum -= energyBuf[energyIndex];
  energyBuf[energyIndex] = energy;
  energySum += energy;
  energyIndex++;
  if (energyIndex >= ENERGY_MA_LEN) energyIndex = 0;

  qrsSignal = (int)(energySum / ENERGY_MA_LEN);

  // ----- Adaptive threshold init/update -----
  if (!qrsThreshInit) {
    static int initCount = 0;
    qrsPeakAvg += qrsSignal;
    initCount++;
    if (initCount >= 50) { // ~0.5 s of data
      qrsPeakAvg /= (float)initCount;
      qrsThreshold = (int)(0.5f * qrsPeakAvg);
      if (qrsThreshold < 50)   qrsThreshold = 50;
      if (qrsThreshold > 2000) qrsThreshold = 2000;
      qrsThreshInit = true;
    }
  } else {
    if (qrsSignal > qrsThreshold &&
        !ecgWasAboveThreshold &&
        (now - ecgLastBeatTime) > ECG_REFRACTORY_MS) {

      // (You could store RR intervals here if you want later)

      ecgLastBeatTime = now;
      ecgWasAboveThreshold = true;

      // Mark this sample as ECG beat
      ecgBeatMarker = 1023;

      // Update running average & threshold
      qrsPeakAvg = 0.9f * qrsPeakAvg + 0.1f * (float)qrsSignal;
      qrsThreshold = (int)(0.5f * qrsPeakAvg);
      if (qrsThreshold < 50)   qrsThreshold = 50;
      if (qrsThreshold > 2000) qrsThreshold = 2000;
    }
    else if (qrsSignal < (qrsThreshold * 0.5f)) {
      ecgWasAboveThreshold = false;
    }
  }

  // ---------- STREAM TO SERIAL PLOTTER ----------
  // Scale QRS to fit roughly 0..1023
  int qrsScaled = qrsSignal / 20;
  if (qrsScaled > 1023) qrsScaled = 1023;

  Serial.print(ppg);            Serial.print(" ");
  Serial.print(ppgBeatMarker);  Serial.print(" ");
  Serial.print(ecg);            Serial.print(" ");
  Serial.print(qrsScaled);      Serial.print(" ");
  Serial.println(ecgBeatMarker);
}
