#include <math.h>

// --------- Pin definitions ----------
const int PPG_PIN   = A0;          // Pulse sensor
const int ECG_PIN   = A1;          // AD8232 OUTPUT
const int LED_PIN   = LED_BUILTIN; // LED for PPG beats (optional)

// If you wired LO+ / LO-:
const int LO_PLUS   = 10;          // AD8232 LO+
const int LO_MINUS  = 11;          // AD8232 LO-

// --------- Sampling & window ----------
const unsigned long SAMPLE_INTERVAL_MS = 10;      // 100 Hz
const unsigned long FEATURE_WINDOW_MS  = 10000;   // 10 seconds

unsigned long lastSampleTime  = 0;
unsigned long windowStartTime = 0;

bool collecting = false;

// ---------------- PPG beat data ----------------
const int MAX_PPG_BEATS = 50;

float ppgIBI[MAX_PPG_BEATS];       // ms
float ppgAmp[MAX_PPG_BEATS];       // ADC amplitude
int   ppgBeatCount = 0;

unsigned long ppgLastBeatTime = 0;
bool  ppgWasAboveThreshold    = false;

int   ppgThreshold = 620;          // TUNE THIS with Serial Plotter (PPG)

// For amplitude between beats
int   ppgMinSinceLastBeat = 1023;
int   ppgMaxSinceLastBeat = 0;

const unsigned long PPG_REFRACTORY_MS = 250;  // avoid double peaks

// Store PPG beat times (for PAT)           // <<< NEW
unsigned long ppgBeatTime[MAX_PPG_BEATS];     // time of PPG peaks (ms)

// ---------------- ECG beat data ----------------
const int MAX_ECG_BEATS = 50;

float ecgRR[MAX_ECG_BEATS];        // ms
int   ecgBeatCount = 0;

unsigned long ecgLastBeatTime = 0;
bool  ecgWasAboveThreshold    = false;

// Store ECG R-peak times (for PAT)        // <<< NEW
unsigned long ecgBeatTime[MAX_ECG_BEATS];     // time of R-peaks (ms)

// -------- QRS enhancement (derivative^2 + moving average) --------
const int ENERGY_MA_LEN = 8;        // length of moving average window
long energyBuf[ENERGY_MA_LEN];
byte energyIndex = 0;
long energySum = 0;

int  prevEcgSample = 0;             // for derivative
int  qrsSignal = 0;                 // processed QRS-energy signal

// Adaptive threshold on qrsSignal
int   qrsThreshold = 0;
bool  qrsThreshInit = false;
float qrsPeakAvg = 0.0;

const unsigned long ECG_REFRACTORY_MS = 400;   // robust up to ~150 bpm

// ---------------- PAT (ECG→PPG delay) -----------------  // <<< NEW
float patValues[MAX_PPG_BEATS];                      // PAT in ms
int   patCount = 0;

// ===================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);

  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("PPG + ECG Extraction (QRS-based) + PAT: press 's' to start, 'x' to stop.");
}

void loop() {
  // -------- Serial commands --------
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      collecting = true;
      resetWindowData();
      windowStartTime = millis();
      Serial.println("Started.");
    } 
    else if (c == 'x' || c == 'X') {
      collecting = false;
      Serial.println("Stopped.");
    }
  }

  unsigned long now = millis();

  // -------- Sampling at 100 Hz --------
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime += SAMPLE_INTERVAL_MS;
    if (collecting) {
      processSample();
    }
  }

  // -------- 10 s window finished --------
  if (collecting && (now - windowStartTime >= FEATURE_WINDOW_MS)) {
    computeFeatures();
    resetWindowData();
    windowStartTime = now;  // start next 10 s window automatically
  }
}

// ===================================================================
//                 SAMPLE PROCESSING (PPG + ECG)
// ===================================================================
void processSample() {
  unsigned long now = millis();

  // ----- Read PPG -----
  int ppg = analogRead(PPG_PIN);

  // Track min/max between beats for amplitude
  if (ppg < ppgMinSinceLastBeat) ppgMinSinceLastBeat = ppg;
  if (ppg > ppgMaxSinceLastBeat) ppgMaxSinceLastBeat = ppg;

  // LED feedback for PPG
  digitalWrite(LED_PIN, (ppg > ppgThreshold));

  // ----- PPG peak detection -----
  if (ppg > ppgThreshold &&
      !ppgWasAboveThreshold &&
      (now - ppgLastBeatTime) > PPG_REFRACTORY_MS) {

    if (ppgLastBeatTime > 0 && ppgBeatCount < MAX_PPG_BEATS) {
      unsigned long ibi = now - ppgLastBeatTime;
      ppgIBI[ppgBeatCount] = (float)ibi;

      int amp = ppgMaxSinceLastBeat - ppgMinSinceLastBeat;
      ppgAmp[ppgBeatCount] = (float)amp;

      // store PPG beat time for this IBI/amplitude          // <<< NEW
      ppgBeatTime[ppgBeatCount] = now;
      
      ppgBeatCount++;
    }

    ppgLastBeatTime = now;
    ppgWasAboveThreshold = true;

    ppgMinSinceLastBeat = ppg;
    ppgMaxSinceLastBeat = ppg;
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
  int diff = ecg - prevEcgSample;          // derivative
  prevEcgSample = ecg;

  long energy = (long)diff * (long)diff;   // "energy" of slope

  // Update moving average buffer
  energySum -= energyBuf[energyIndex];
  energyBuf[energyIndex] = energy;
  energySum += energy;
  energyIndex++;
  if (energyIndex >= ENERGY_MA_LEN) energyIndex = 0;

  qrsSignal = (int)(energySum / ENERGY_MA_LEN);   // smoothed energy

  // ----- Initialize / update adaptive threshold -----
  if (!qrsThreshInit) {
    // Simple bootstrap for first few samples
    static int initCount = 0;
    qrsPeakAvg += qrsSignal;
    initCount++;
    if (initCount >= 50) {                  // after ~0.5 s
      qrsPeakAvg /= (float)initCount;
      qrsThreshold = (int)(0.5f * qrsPeakAvg);   // 50% of avg energy
      if (qrsThreshold < 50)  qrsThreshold = 50;
      if (qrsThreshold > 2000) qrsThreshold = 2000;
      qrsThreshInit = true;
    }
    return;  // don't detect beats until threshold is ready
  }

  // ----- QRS detection using qrsSignal -----
  if (qrsSignal > qrsThreshold &&
      !ecgWasAboveThreshold &&
      (now - ecgLastBeatTime) > ECG_REFRACTORY_MS) {

    if (ecgLastBeatTime > 0 && ecgBeatCount < MAX_ECG_BEATS) {
      unsigned long rr = now - ecgLastBeatTime;
      ecgRR[ecgBeatCount] = (float)rr;

      // store ECG R-peak time for this RR interval       // <<< NEW
      ecgBeatTime[ecgBeatCount] = now;

      ecgBeatCount++;
    }

    ecgLastBeatTime = now;
    ecgWasAboveThreshold = true;

    // Update running average of QRS energy and threshold
    qrsPeakAvg = 0.9f * qrsPeakAvg + 0.1f * (float)qrsSignal;
    qrsThreshold = (int)(0.5f * qrsPeakAvg);      // keep at ~50% of typical QRS
    if (qrsThreshold < 50)  qrsThreshold = 50;
    if (qrsThreshold > 2000) qrsThreshold = 2000;
  }
  else if (qrsSignal < (qrsThreshold * 0.5f)) {
    // reset detector once energy has fallen well below threshold
    ecgWasAboveThreshold = false;
  }

  // If you want to debug, you can print qrsSignal vs raw ECG:
  // Serial.print(ecg); Serial.print(" "); Serial.println(qrsSignal);
}

// ===================================================================
//                        FEATURE COMPUTATION
// ===================================================================
void computeFeatures() {
  Serial.println("--------------------------------------------------");
  Serial.println("10-second window finished. Features:");

  // ---------- PPG features ----------
  Serial.println("PPG features:");

  Serial.print("  Beats detected: ");
  Serial.println(ppgBeatCount);

  if (ppgBeatCount >= 2) {
    float sumIBI = 0;
    for (int i = 0; i < ppgBeatCount; i++) sumIBI += ppgIBI[i];
    float ibiMean = sumIBI / ppgBeatCount;

    float sumVar = 0;
    for (int i = 0; i < ppgBeatCount; i++) {
      float diff = ppgIBI[i] - ibiMean;
      sumVar += diff * diff;
    }
    float ibiVar = sumVar / ppgBeatCount;

    float hrMean = 60000.0 / ibiMean;

    float sumAmp = 0;
    for (int i = 0; i < ppgBeatCount; i++) sumAmp += ppgAmp[i];
    float ampMean = sumAmp / ppgBeatCount;

    Serial.print("  IBI mean (ms):      "); Serial.println(ibiMean, 2);
    Serial.print("  IBI variance:       "); Serial.println(ibiVar, 2);
    Serial.print("  HR mean (bpm):      "); Serial.println(hrMean, 2);
    Serial.print("  Amp mean (ADC):     "); Serial.println(ampMean, 2);
  } else {
    Serial.println("  Not enough PPG beats.");
  }

  // ---------- ECG features ----------
  Serial.println("ECG features:");

  Serial.print("  Beats detected: ");
  Serial.println(ecgBeatCount);

  if (ecgBeatCount >= 2) {
    float rrMean = 0;
    for (int i = 0; i < ecgBeatCount; i++) rrMean += ecgRR[i];
    rrMean /= ecgBeatCount;

    // SDNN
    float sumSq = 0;
    for (int i = 0; i < ecgBeatCount; i++) {
      float diff = ecgRR[i] - rrMean;
      sumSq += diff * diff;
    }
    float sdnn = sqrt(sumSq / ecgBeatCount);

    // RMSSD and pNN50
    float sumSqDiff = 0;
    int   nn50 = 0;
    int   nDiff = ecgBeatCount - 1;
    for (int i = 0; i < nDiff; i++) {
      float diff = ecgRR[i+1] - ecgRR[i];
      float absDiff = fabs(diff);
      sumSqDiff += diff * diff;
      if (absDiff > 50.0) nn50++;
    }
    float rmssd = sqrt(sumSqDiff / nDiff);
    float pnn50 = (float)nn50 * 100.0 / (float)nDiff;

    float hrMeanECG = 60000.0 / rrMean;

    Serial.print("  RR mean (ms):       "); Serial.println(rrMean, 2);
    Serial.print("  SDNN (ms):          "); Serial.println(sdnn, 2);
    Serial.print("  RMSSD (ms):         "); Serial.println(rmssd, 2);
    Serial.print("  pNN50 (%):          "); Serial.println(pnn50, 2);
    Serial.print("  HR mean (bpm):      "); Serial.println(hrMeanECG, 2);
  } else {
    Serial.println("  Not enough ECG beats.");
  }

  // ---------- PAT features (ECG R → PPG peak) ----------   // <<< NEW
  // PAT ≈ Pulse Arrival Time: delay between R-peak and the following PPG peak
  patCount = 0;

  int j = 0; // index in ECG beat times
  for (int i = 0; i < ppgBeatCount && patCount < MAX_PPG_BEATS; i++) {
    unsigned long tPPG = ppgBeatTime[i];

    // advance j until ecgBeatTime[j] >= tPPG
    while (j < ecgBeatCount && ecgBeatTime[j] < tPPG) {
      j++;
    }
    if (j == 0) {
      // no ECG beat before this PPG beat
      continue;
    }

    unsigned long tECG = ecgBeatTime[j - 1];  // last R-peak before this PPG peak
    unsigned long dt = tPPG - tECG;          // PAT in ms

    // keep only physiologically reasonable PAT values (e.g. 100–400 ms)
    if (dt >= 100 && dt <= 400) {
      patValues[patCount] = (float)dt;
      patCount++;
    }
  }

  Serial.println("PAT (ECG R -> PPG peak) features:");
  Serial.print("  Valid PAT count: ");
  Serial.println(patCount);

  if (patCount >= 1) {
    float sumPAT = 0;
    for (int i = 0; i < patCount; i++) sumPAT += patValues[i];
    float patMean = sumPAT / patCount;

    float sumPATVar = 0;
    for (int i = 0; i < patCount; i++) {
      float diff = patValues[i] - patMean;
      sumPATVar += diff * diff;
    }
    float patVar = sumPATVar / patCount;

    Serial.print("  PAT mean (ms):      "); Serial.println(patMean, 2);
    Serial.print("  PAT variance:       "); Serial.println(patVar, 2);
  } else {
    Serial.println("  Not enough valid PAT pairs.");
  }

  Serial.println("--------------------------------------------------");
}

// ===================================================================
//                        RESET DATA
// ===================================================================
void resetWindowData() {
  // PPG
  ppgBeatCount = 0;
  ppgLastBeatTime = 0;
  ppgWasAboveThreshold = false;
  ppgMinSinceLastBeat = 1023;
  ppgMaxSinceLastBeat = 0;

  // ECG
  ecgBeatCount = 0;
  ecgLastBeatTime = 0;
  ecgWasAboveThreshold = false;

  // QRS processing
  prevEcgSample = 0;
  energySum = 0;
  for (int i = 0; i < ENERGY_MA_LEN; i++) energyBuf[i] = 0;
  energyIndex = 0;
  qrsThreshInit = false;
  qrsPeakAvg = 0.0;
  qrsThreshold = 0;

  // PAT
  patCount = 0;
}
