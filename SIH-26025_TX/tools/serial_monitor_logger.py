import os
import sys
import time
import serial

PORT = "COM3"  # Adjust for your ESP32 port (e.g. '/dev/ttyUSB0' on Linux/Mac)
BAUD = 115200
OUTPUT_FILE = os.path.join("data", "normal_log.csv")

CSV_HEADER = (
    "displacementX_mm,displacementY_mm,displacementZ_mm,"
    "accX_mmss2,accY_mmss2,accZ_mmss2,resultantAccel,"
    "rateRollDegS,ratePitchDegS,rateYawDegS,"
    "angleRoll,anglePitch,angleYaw,vibrationDetected,isHighPrecision\n"
)

os.makedirs("data", exist_ok=True)

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print(f"--- Connected to {PORT} at {BAUD} baud ---")
except Exception as e:
    sys.exit(f"Error opening port {PORT}: {e}")

# Opening with 'w' wipes/cleans old data automatically
with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    f.write(CSV_HEADER)
    f.flush()
    print(f"--- Wiped old file and created clean log at {OUTPUT_FILE} ---")
    print("--- Live Stream below (Press Ctrl+C to stop) ---\n")

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    # 1. Print to screen (Acts as Serial Monitor)
                    print(line)

                    # 2. Log valid 15-feature CSV rows to file in background
                    parts = line.split(",")
                    if len(parts) == 15 and all(p.lstrip("-").isdigit() for p in parts):
                        f.write(line + "\n")
                        f.flush()
    except KeyboardInterrupt:
        print("\n--- Stopped logging & monitoring ---")
    finally:
        ser.close()