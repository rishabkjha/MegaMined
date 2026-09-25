import os
import re
import sys
import time
import serial

PORT = "COM18"  # Adjust for your ESP32 port
BAUD = 115200
OUTPUT_FILE = os.path.join("data", "normal_log.csv")

CSV_HEADER = (
    "displacementX_mm,displacementY_mm,displacementZ_mm,"
    "accX_mmss2,accY_mmss2,accZ_mmss2,resultantAccel,"
    "rateRollDegS,ratePitchDegS,rateYawDegS,"
    "angleRoll,anglePitch,angleYaw,vibrationDetected,isHighPrecision\n"
)

os.makedirs("data", exist_ok=True)


def is_valid_numeric_row(parts, expected_count=15):
    """Validates if a list of items consists of exactly 15 numbers."""
    if len(parts) != expected_count:
        return False
    for item in parts:
        try:
            float(item.strip())
        except ValueError:
            return False
    return True


# Safely attempt to open the serial port
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"--- Connected to {PORT} at {BAUD} baud ---")
except serial.SerialException as e:
    print(f"\n[WARNING] Could not open port {PORT}: {e}")
    print("[WARNING] Skipping live logging. If PlatformIO is building, close other serial monitors.")
    sys.exit(0)  # Exit cleanly so SCons/PlatformIO build isn't killed

# Buffer to accumulate structured block data
current_data = {}

with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    f.write(CSV_HEADER)
    f.flush()
    print(f"--- Initialized clean log at {OUTPUT_FILE} ---")
    print("--- Live Stream below (Press Ctrl+C to stop) ---\n")

    logged_count = 0

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Always display output in serial monitor
            print(line)

            # ----------------------------------------------------
            # MODE 1: Directly handle raw comma-separated CSV lines
            # ----------------------------------------------------
            parts = line.split(",")
            if is_valid_numeric_row(parts, 15):
                f.write(line + "\n")
                f.flush()
                logged_count += 1
                print(f"  --> [SAVED LOG #{logged_count}]")
                current_data = {}  # Clear dictionary buffer
                continue

            # ----------------------------------------------------
            # MODE 2: Parse multi-line UI debug blocks
            # ----------------------------------------------------
            if "High Precision Active" in line:
                current_data["isHighPrecision"] = 1 if "YES" in line else 0

            elif "Displacement (X, Y, Z):" in line:
                match = re.search(r"X:\s*(\S+)\s*mm\s*\|\s*Y:\s*([\d\.-]+)\s*mm\s*\|\s*Z:\s*([\d\.-]+)", line)
                if match:
                    current_data["displacementX_mm"] = 0.0 if "ovf" in match.group(1) else float(match.group(1))
                    current_data["displacementY_mm"] = float(match.group(2))
                    current_data["displacementZ_mm"] = float(match.group(3))

            elif "Angles (R/P/Y)" in line:
                match = re.search(r"R:\s*([\d\.-]+)\s*\|\s*P:\s*([\d\.-]+)\s*\|\s*Y:\s*([\d\.-]+)", line)
                if match:
                    current_data["angleRoll"] = float(match.group(1))
                    current_data["anglePitch"] = float(match.group(2))
                    current_data["angleYaw"] = float(match.group(3))

            elif "Gyro Rates (R/P/Y)" in line:
                match = re.search(r"R:\s*([\d\.-]+)\s*\|\s*P:\s*([\d\.-]+)\s*\|\s*Y:\s*([\d\.-]+)", line)
                if match:
                    current_data["rateRollDegS"] = float(match.group(1))
                    current_data["ratePitchDegS"] = float(match.group(2))
                    current_data["rateYawDegS"] = float(match.group(3))

            elif "Accelerations (X/Y/Z)" in line:
                match = re.search(r"X:\s*([\d\.-]+)\s*\|\s*Y:\s*([\d\.-]+)\s*\|\s*Z:\s*([\d\.-]+)\s*mm/s\^2\s*\|\s*Resultant:\s*([\d\.-]+)", line)
                if match:
                    current_data["accX_mmss2"] = float(match.group(1))
                    current_data["accY_mmss2"] = float(match.group(2))
                    current_data["accZ_mmss2"] = float(match.group(3))
                    current_data["resultantAccel"] = float(match.group(4))

            elif "Vibration Status" in line:
                current_data["vibrationDetected"] = 1 if "VIBRATION" in line else 0

            # Write out block once all 15 fields have been populated
            elif "================" in line and len(current_data) >= 15:
                row_str = (
                    f"{current_data['displacementX_mm']},{current_data['displacementY_mm']},{current_data['displacementZ_mm']},"
                    f"{current_data['accX_mmss2']},{current_data['accY_mmss2']},{current_data['accZ_mmss2']},{current_data['resultantAccel']},"
                    f"{current_data['rateRollDegS']},{current_data['ratePitchDegS']},{current_data['rateYawDegS']},"
                    f"{current_data['angleRoll']},{current_data['anglePitch']},{current_data['angleYaw']},"
                    f"{current_data['vibrationDetected']},{current_data['isHighPrecision']}\n"
                )
                f.write(row_str)
                f.flush()
                logged_count += 1
                print(f"  --> [SAVED LOG #{logged_count}]")
                current_data = {}  # Reset buffer

    except KeyboardInterrupt:
        print(f"\n--- Stopped logging. Total rows saved: {logged_count} ---")
    finally:
        ser.close()