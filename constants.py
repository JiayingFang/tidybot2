import numpy as np

################################################################################
# Mobile base

# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# Encoder magnet offsets
ENCODER_MAGNET_OFFSETS = [1465.0 / 4096, 175.0 / 4096, 1295.0 / 4096, 1595.0 / 4096]  # Base #1 (IPRL Kinova)

################################################################################
# Teleop and imitation learning

# Base and arm RPC servers
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 50000
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 50001
RPC_AUTHKEY = b'secret password'

# Cameras
BASE_CAMERA_SERIAL = 'DDCD281E'  # Base #1
# WRIST_CAMERA_SERIAL = 'TODO'  # Not used by Kinova wrist camera

# Policy
#POLICY_SERVER_HOST = '172.24.69.76' #'172.24.69.31' #'10.34.161.79' # # R2D2 Laptop
POLICY_SERVER_HOST = "172.24.69.31"
POLICY_SERVER_PORT = 5555
POLICY_CONTROL_FREQ = 10
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ
POLICY_IMAGE_WIDTH = 84
POLICY_IMAGE_HEIGHT = 84


EE_OFFSET = 0.148