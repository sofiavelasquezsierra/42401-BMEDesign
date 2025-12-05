# PPG Calibration and Model Training

Here's how to calibrate the MAX30102 PPG sensor and train ML model to estimate HR and SpO2 from raw sensor data:

1. Upload Arduino calibration firmware
2. Collect calibration data with a fingertip pulse oximeter
3. Train the ML models and run live inference

---

## 1. Upload the Calibration Firmware

Upload file to MCU:

    ppg_calibration.ino

The firmware outputs raw MAX30102 readings in format:

    IR=53422,RED=19100,IR_AC=220,IR_DC=54000,RED_AC=85,RED_DC=19500,R=0.562341,T=123456

---

## 2. Run Calibration Logger

Put fingertip oximeter on and tape MAX30102 to another finger on the same hand.

Start calibration with:
```
python calibration.py
```
This script:

- Asks for true HR and SpO2 values from fingertip pulse oximeter
- Records raw PPG values for a fixed number of seconds per calibration set
- Saves all collected data into `ppg_calibration_data.csv`

### Recommended Calibration Procedure

Each calibration set is typically 2 seconds, giving ~200 samples at 100 Hz.

To build a robust model, you should collect data in a variety of physiological and motion conditions.

#### Important Notes Before You Begin
- Make sure MAX30102 is securely attached to finger, especially for movement exercises.
- After each movement or breathing exercise, **wait for heart rate and SpO₂ on the clinical oximeter to stabilize** before starting next 2-second calibration set.
- HR may take **5–20 seconds** to return to baseline after walking or jogging.
- SpO₂ may lag by **5–10 seconds**, especially after breath-holding.
- Only start the next calibration set **when the clinical oximeter shows a stable value**.

Recommended conditions:

Condition | Purpose | Suggested sets
--------- | -------- | --------------
Sitting | baseline | 20
Standing | altered circulation | 20
Arm raised | low perfusion | 15
Talking / small motion | mild motion artifacts | 15
Holding breath | slight desaturation | 15
Walking | motion artifacts | 20
Light jogging / stepping | increased HR | 20
Deep breathing | HR variability | 15

Suggested total calibration sets: 170

You can modify these values in `calibration.py`:

    CALIBRATION_SETS = 170
    SECONDS_PER_SET = 2

---

## 3. Train ML Models

Once calibration data is collected, run:
```
python calibration_train.py
```
This script:

- Loads `ppg_calibration_data.csv`
- Filters out invalid readings
- Trains several regression models (Polynomial, Random Forest, Gradient Boosting)
- Picks best-performing model for HR and SpO2
- Saves trained models to:
    hr_model.pkl
    spo2_model.pkl

---

## 4. Run Live Inference

To test trained models in real time:
```
python live_inference.py
```
This script reads raw values from sensor and prints:

    HR = xx.x bpm
    SpO2 = xx.x %

Optionally logs live predictions to CSV.
