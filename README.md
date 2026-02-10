##  Technical Architecture

We use a **Hybrid Two-Model Pipeline**. This architecture ensures that high heart rates caused by physical exertion (running) are not misclassified as medical emergencies (shock).

### Model A: Activity Classifier
- **Sensor Input:** 3-axis Accelerometer (LSM6DS3TR-C)
- **Purpose:** Classifies the soldier's movement state
- **Labels:** `lying_still`, `walking`, `running`, `falling`
- **Data Source:** Trained on the PPG-DaLiA dataset

### Model B: Triage Classifier
- **Sensor Input:** Vitals (HR, HRV, SpO2, Breathing Rate) + Output from Model A
- **Purpose:** Assigns the final medical triage category
- **Labels:** T1 (Immediate), T2 (Delayed), T3 (Minimal/Healthy)
- **Data Source:** Trained on PhysioNet MIMIC-III/IV clinical trauma data

---

## 3. The "Virtual Prototype" (Python Development)

Before moving to hardware, we prove the logic in a Python-based simulation environment.

### Feature Extraction (`feature_extractor.py`)
- **ECG:** Processed via `NeuroKit2` to extract Heart Rate (HR) and HRV (RMSSD)
- **Respiration:** Calculated via ECG-Derived Respiration (EDR)
- **PPG:** Red/IR signals are filtered to calculate Blood Oxygen (SpO2)
- **IMU:** Raw data is converted into time-domain features (Mean, Variance, Magnitude)

### ML Model Logic
- **Algorithm:** Random Forest Classifier
- **Rationale:** Low power consumption, high interpretability, and easily converted to C++ code
- **Key Heuristic:** The model learns that high vitals are Normal (T3) if the activity is `running`, but Critical (T1) if the activity is `lying_still` and SpO2 is dropping

---

## 4. Hardware Integration Roadmap (Next Steps)

The next phase is translating the Python logic into C++ for the Arduino IDE.

### I. Firmware: The Data Logger
1. Initialize I2C/Analog communication for the AD8232, MAXREFDES117, and LSM6DS3
2. Log raw, timestamped sensor data to the 64GB MicroSD Card
3. **Goal:** Create a "Real-World" dataset to validate the Python-trained models

### II. Model Conversion (Python to C++)
Since the nRF52840 cannot run Python scripts:
- Use `micromlgen` or Edge Impulse to convert the trained `.pkl` models into C++ header files (`.h`)
- Include these headers in the Arduino sketch: `#include "triage_model.h"`
