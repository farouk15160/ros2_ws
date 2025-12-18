import os
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory

def get_firmware_path():
    pkg_path = get_package_share_directory('visiona_firmware')
    firmware_path = os.path.join(pkg_path, 'firmware')
    
    if not os.path.exists(firmware_path):
        print(f"Error: Firmware directory not found at {firmware_path}")
        sys.exit(1)
        
    return firmware_path

def flash():
    firmware_path = get_firmware_path()
    print(f"Flashing firmware from: {firmware_path}")
    print("Running: pio run -t upload")
    
    try:
        subprocess.check_call(['pio', 'run', '-t', 'upload'], cwd=firmware_path)
    except subprocess.CalledProcessError as e:
        print(f"Flash failed: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: 'pio' command not found. Please install PlatformIO (pip install platformio).")
        sys.exit(1)

def monitor():
    firmware_path = get_firmware_path()
    print(f"Monitoring firmware from: {firmware_path}")
    print("Running: pio device monitor")
    
    try:
        subprocess.check_call(['pio', 'device', 'monitor'], cwd=firmware_path)
    except subprocess.CalledProcessError as e:
        print(f"Monitor failed: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: 'pio' command not found. Please install PlatformIO (pip install platformio).")
        sys.exit(1)
