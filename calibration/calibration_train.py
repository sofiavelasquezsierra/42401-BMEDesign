# calibration_train.py
# Load calibration data collected from MAX30102 (PPG) sensor and train ML models
# Map raw PPG features (IR, RED, AC/DC components, and R ratio) to heart rate and SpO₂
# Output two trained regression models: hr_model.pkl and spo2_model.pkl
#
# Feature meanings:
#   ir       – raw infrared photodiode reading (reflected IR light intensity)
#   red      – raw red photodiode reading (reflected red light intensity)
#   ir_ac    – pulsatile IR amplitude (peak-to-trough caused by heartbeat)
#   red_ac   – pulsatile red amplitude
#   ir_dc    – baseline IR level (slow-changing tissue absorption)
#   red_dc   – baseline red level
#   R        – (red_ac/red_dc) / (ir_ac/ir_dc), the standard “ratio of ratios” used in pulse oximetry

import pandas as pd
import numpy as np
import joblib
import copy
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.ensemble import RandomForestRegressor, GradientBoostingRegressor
from sklearn.linear_model import LinearRegression

INPUT_FILE = "ppg_calibration_data.csv"

# Polynomial regression model builder
def polynomial_model(degree=2):
    return Pipeline([
        ("poly", PolynomialFeatures(degree, include_bias=False)),
        ("lin", LinearRegression())
    ])

# Load calibration dataset
df = pd.read_csv(INPUT_FILE)

print("Loaded rows:", len(df))
print("Columns:", df.columns.tolist())

# Filter physically invalid readings
df = df[(df["ir"] > 0) & (df["red"] > 0)]
df = df[(df["ir_ac"] > 0) & (df["red_ac"] > 0)]
df = df[(df["ir_dc"] > 0) & (df["red_dc"] > 0)]

print("After filtering invalid rows:", len(df))

if len(df) < 30:
    raise ValueError("Not enough usable calibration samples.")

# Select model features
FEATURES = ["ir", "red", "ir_ac", "ir_dc", "red_ac", "red_dc", "r_value"]
X = df[FEATURES].values

# Targets are clinical HR and SpO₂ values recorded during calibration
y_hr = df["true_hr"].values
y_spo2 = df["true_spo2"].values

# Several models to try; choose whichever performs best
models = {
    "poly2": polynomial_model(2),
    "poly3": polynomial_model(3),
    "rf": RandomForestRegressor(n_estimators=300, random_state=42),
    "gbr": GradientBoostingRegressor(random_state=42)
}

# Train and evaluate each model type
def evaluate_model(model, X_train, X_test, y_train, y_test, name):
    model.fit(X_train, y_train)
    pred = model.predict(X_test)

    mae = mean_absolute_error(y_test, pred)
    rmse = np.sqrt(mean_squared_error(y_test, pred))
    r2 = r2_score(y_test, pred)

    print(f"\n{name} Performance:")
    print(f"  MAE:  {mae:.2f}")
    print(f"  RMSE: {rmse:.2f}")
    print(f"  R²:   {r2:.3f}")

    return pred, rmse

# Train models for either HR or SpO₂, compare performance, select best one
def train_target(X, y, label):
    print(f"\nTraining models for {label}")

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, shuffle=True, random_state=42
    )

    best_model = None
    best_rmse = float("inf")
    best_pred = None
    best_y_test = y_test

    for name, model in models.items():
        m = copy.deepcopy(model)
        pred, rmse = evaluate_model(
            m, X_train, X_test, y_train, y_test, f"{label} ({name})"
        )

        if rmse < best_rmse:
            best_rmse = rmse
            best_model = m
            best_pred = pred

    # Plot predicted vs actual values for best model
    plt.figure(figsize=(6, 6))
    plt.scatter(best_y_test, best_pred, alpha=0.5)
    plt.plot(
        [min(best_y_test), max(best_y_test)],
        [min(best_y_test), max(best_y_test)],
        "r--"
    )
    plt.xlabel("True")
    plt.ylabel("Predicted")
    plt.title(f"{label} — Best Model Fit (RMSE={best_rmse:.2f})")
    plt.grid()
    plt.show()

    return best_model

# Train HR and SpO₂ models
best_hr_model = train_target(X, y_hr, "Heart Rate")
best_spo2_model = train_target(X, y_spo2, "SpO2")

# Save trained models for inference
joblib.dump(best_hr_model, "hr_model.pkl")
joblib.dump(best_spo2_model, "spo2_model.pkl")

print("\nSaved models:")
print("  hr_model.pkl")
print("  spo2_model.pkl")
