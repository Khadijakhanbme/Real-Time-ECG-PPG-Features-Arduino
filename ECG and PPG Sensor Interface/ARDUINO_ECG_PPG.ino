// --------- Pin definitions ----------
const int PPG_PIN   = A0;        // Pulse sensor (purple wire)
const int ECG_PIN   = A1;        // AD8232 OUTPUT pin
const int LED       = LED_BUILTIN;

const int LO_PLUS   = 10;        // AD8232 LO+
const int LO_MINUS  = 11;        // AD8232 LO-

// --------- Variables ----------
int Threshold = 580;             // PPG threshold for LED beat indication

void setup() {
  pinMode(LED, OUTPUT);

  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);

  Serial.begin(115200);          // Use 115200 baud in Serial Plotter
}

void loop() {
  // ----- Read PPG (pulse sensor) -----
  int ppgSignal = analogRead(PPG_PIN);

  // LED on each heartbeat (same as your first code)
  if (ppgSignal > Threshold) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }

  // ----- Read ECG (AD8232) -----
  int ecgSignal = analogRead(ECG_PIN);

  // If leads are off, force ECG line to 0 so the plot is clear
  if (digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH) {
    ecgSignal = 0;
  }

  // ----- Send BOTH to Serial Plotter -----
  // Serial Plotter will show 2 lines: PPG and ECG
  Serial.print("PPG:");
  Serial.print(ppgSignal);
  Serial.print(" ECG:");
  Serial.println(ecgSignal);

  delay(20);   // ~50 samples/second for both
}
