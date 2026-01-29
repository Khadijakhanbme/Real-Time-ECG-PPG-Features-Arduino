# Real-Time ECG–PPG Feature Extraction (Arduino) — Stress Detection Path

This repository contains the **Arduino-based real-time signal acquisition and feature extraction** pipeline for a multimodal **ECG + PPG stress detection** prototype.

It represents the **Real-Time Path** in the overall workflow, where ECG and PPG signals are acquired from sensors, peaks are detected on-device, and **features are computed every 10 seconds** for downstream inference (external model / dashboard).

---

## Purpose

The goal of this repo is to provide a **working embedded pipeline** to:

- acquire **ECG (AD8232)** and **PPG (pulse sensor)** signals in real time,
- perform **robust peak detection** (ECG R-peaks + PPG peaks),
- compute physiological features per **10-second window** (aligned with the prototype requirements),
- optionally compute **Pulse Arrival Time (PAT)** as ECG→PPG delay,
- stream signals and detection markers for **visual inspection** via Serial Plotter.

This implementation was designed for a real-time experimental setup where **0-back** (baseline / lower workload) and **2-back** (higher workload) conditions were used as the two classes for **stress / no-stress comparison** (1-back was not used in this prototype setup).

---

## Hardware Setup

- **Arduino Mega**
- **ECG sensor:** AD8232 (OUTPUT → A1, LO+/LO- optional)
- **PPG sensor:** Pulse sensor (signal → A0)
- Serial output used for:
  - live visualization (Serial Plotter),
  - streaming values for logging / dashboard integration.

---

## What’s Included

### 1) Initial Signal Check (ECG + PPG)
A minimal sketch to confirm that both signals are being acquired and plotted.

- Reads ECG + PPG
- Prints both channels to Serial Plotter
- Useful for verifying wiring and sensor placement

### 2) Continuous Visualization + Peak Markers (Debug/Validation)
A visualization-focused sketch to validate that:
- PPG thresholding is working,
- QRS-enhancement for ECG helps detect R-peaks,
- beat markers appear correctly.

It streams:
- PPG
- PPG beat marker
- ECG
- QRS processed signal (scaled)
- ECG beat marker

This step was used to ensure signal quality and correct peak detection **before** final feature extraction.

### 3) Real-Time Feature Extraction (10-second windows)
A full real-time feature extraction sketch that:
- segments incoming data into **10-second windows**,
- detects peaks with refractory control,
- computes features per window for both ECG and PPG,
- computes PAT (ECG R-peak → following PPG peak) when valid.

---

## Extracted Features (per 10-second window)

### PPG
- Mean inter-beat interval (IBI)
- IBI variance
- Mean heart rate (from PPG)
- Mean pulse amplitude (ADC range between beats)

### ECG (HRV-style time-domain)
- Mean RR interval
- Mean heart rate (from ECG)
- SDNN
- RMSSD
- pNN50

### PAT (ECG → PPG delay)
- PAT mean (ms)
- PAT variance
- Uses physiologically valid PAT range (e.g., 100–400 ms)

---

## How to Run

1. Open the desired `.ino` sketch in Arduino IDE.
2. Select the correct board and port (Arduino Mega).
3. Upload the sketch.
4. Open **Serial Plotter** or **Serial Monitor** at `115200 baud`.

**For feature extraction:**
- Send `s` to start collecting
- Send `x` to stop
- Features print automatically after each **10-second window**

---

## Notes

- Threshold values (PPG threshold, QRS threshold behavior) may need tuning depending on sensor placement and participant physiology.
- This repository covers **only the Real-Time Path** (embedded side) of the workflow attached below.
  Offline model training / dataset experiments / classification are maintained in a separate repository (https://github.com/Khadijakhanbme/ECG-PPG-Stress-Analysis)

  <img width="1548" height="729" alt="image" src="https://github.com/user-attachments/assets/f543ea6d-092f-4d3d-9a80-10ffe53b6b81" />


  
