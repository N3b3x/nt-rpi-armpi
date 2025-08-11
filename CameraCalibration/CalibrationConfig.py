import os

# The calibration chessboard size is determined by the number of internal corners, excluding the non-chessboard corners, in both the row and column directions.
calibration_size = (9, 6)

# Base directory of this config file, so paths work regardless of CWD
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Storage path for calibration image acquisition
save_path = os.path.join(_BASE_DIR, 'calib')

# Storage path (prefix) for the calibration parameters (npz without extension)
calibration_param_path = os.path.join(_BASE_DIR, 'calibration_param')
