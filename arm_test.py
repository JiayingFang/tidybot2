#!/home/marion/miniforge3/envs/tidybot2/bin/python
import numpy as np
import time
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY, POLICY_CONTROL_PERIOD
from arm_server import ArmManager
import keyboard


if __name__ == '__main__':
    # manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    # server = manager.get_server()
    # print(f'Arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
    # server.serve_forever()
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    arm = manager.Arm()
    try:
        arm.reset()
        # print(arm.get_state_admittance())
        # arm.enter_admittance()
        # print("Press Enter to continue")
        # while arm:
        #     button_pressed = False
        #     while not button_pressed:
        #         button_pressed = keyboard.is_pressed('enter')
        #     print(arm.get_state_admittance())

        for i in range(100):
            arm.execute_action({
                'arm_pos': np.array([0.15, 0.1, 0.211]),
                'arm_quat': np.array([0.706, 0.707, 0.029, 0.029]),
                'gripper_pos': np.ones(1),
            })
            print(arm.get_state())
            time.sleep(0.1)  # Note: Not precise
    finally:
        arm.close()
