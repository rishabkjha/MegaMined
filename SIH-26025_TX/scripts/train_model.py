import os
import subprocess
import sys

Import("env")

# Auto-install missing python modules into PlatformIO's environment
try:
    import numpy
    import serial  # Added pyserial check for serial_monitor_logger.py
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy", "scikit-learn", "pyserial"])

# Relative paths from the project root (SIH-26025/)
csv_path = "data/normal_log.csv"
logger_script_path = "tools/serial_monitor_logger.py"
train_script_path = "tools/Train_Isolation_Forest.py"
output_header = "include/Isolation_Forest.h"

python_exe = env.subst("$PYTHONEXE")

# Step 1: Run serial monitor & logger if the script exists
if os.path.exists(logger_script_path):
    print(f"[Pre-Build Hook] Launching Serial Monitor & Data Logger ({logger_script_path})...")
    # This will wipe data/normal_log.csv, log incoming serial data, and exit when stopped
    subprocess.run([
        python_exe, logger_script_path
    ], check=True)
else:
    print(f"[Pre-Build Hook] WARNING: {logger_script_path} not found! Skipping serial logging step.")

# Step 2: Retrain Isolation Forest model using the captured CSV
if os.path.exists(csv_path):
    print(f"[Pre-Build Hook] Retraining Isolation Forest model using {csv_path}...")
    subprocess.run([
        python_exe, train_script_path,
        csv_path,
        "-o", output_header,
        "--verify"
    ], check=True)
else:
    print(f"[Pre-Build Hook] WARNING: {csv_path} not found! Skipping model retraining.")