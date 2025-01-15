
import numpy as np
import pdb 
import redis
from constants import POLICY_CONTROL_PERIOD
from redis_utils import encode_matlab, decode_matlab
from arm_server import ArmManager
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
import time
from pyquaternion import Quaternion

TIDYBOT_NUC_HOST = "172.24.69.38"
TIDYBOT_PORT = 6379
TIDYBOT_PWD = ""

KEY_CONTROL_POS = "tidybot::control_pos"
KEY_CONTROL_ORI = "tidybot::control_ori"
KEY_CONTROL_GRIPPER = "tidybot::control_gripper"

KEY_SENSOR_POS = "tidybot::sensor_pos"
KEY_SENSOR_ORI = "tidybot::sensor_ori"
KEY_SENSOR_GRIPPER = "tidybot::sensor_gripper"

DELTA_ACTION_THRESHOLD = 0.2

class HumanShadowKinovaEnv:
    def __init__(self):
        # self.ee_home_pos = np.array([0.3, 0, 0.3])
        # self.ee_home_ori = np.array([1, 0, 0, 0])
        self.ee_home_pos = np.array([0.135, 0.002, 0.211])
        self.ee_home_ori = np.array([0.706, 0.707, 0.029, 0.029])
        self.gripper_home_pos = np.array([0])

        # Initialize redis
        _redis = redis.Redis(
            host=TIDYBOT_NUC_HOST, port=TIDYBOT_PORT, password=TIDYBOT_PWD
        )
        self.redis_pipe = _redis.pipeline()
        self.redis_pipe.set(KEY_CONTROL_POS, encode_matlab(self.ee_home_pos))
        self.redis_pipe.set(KEY_CONTROL_ORI, encode_matlab(self.ee_home_ori))
        self.redis_pipe.set(KEY_CONTROL_GRIPPER, encode_matlab(self.gripper_home_pos))
        self.redis_pipe.execute()

        # Initialize arm
        arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            arm_manager.connect()
        except ConnectionRefusedError as e:
            raise Exception('Could not connect to arm RPC server, is arm_server.py running?') from e
        self.arm = arm_manager.Arm()

    def get_obs(self):
        obs = {}
        obs.update(self.arm.get_state())
        return obs

    def reset(self):
        print('Resetting arm...')
        self.arm.reset()
        print('Robot has been reset')

    def step(self):
        self.redis_pipe.get(KEY_CONTROL_POS)
        self.redis_pipe.get(KEY_CONTROL_ORI)
        self.redis_pipe.get(KEY_CONTROL_GRIPPER)
        b_pos, b_quat, b_gripper = self.redis_pipe.execute()
        pos = decode_matlab(b_pos)
        quat = decode_matlab(b_quat)
        gripper = decode_matlab(b_gripper)
        if not self.is_safe(pos_des=pos, ori_des=quat):
            self.close()
            raise ValueError(f"WARNING: Unsafe command, pos: {pos}, quat: {quat}")

        action = {
            'arm_pos': pos,
            'arm_quat': quat,
            'gripper_pos': gripper,
        }
        print(action)
        self.arm.execute_action(action)   # Non-blocking

        # Update obs 
        state = self.arm.get_state()
        self.redis_pipe.set(KEY_SENSOR_POS, encode_matlab(state["arm_pos"]))
        self.redis_pipe.set(KEY_SENSOR_ORI, encode_matlab(state["arm_quat"]))
        self.redis_pipe.set(KEY_SENSOR_GRIPPER, encode_matlab(state["gripper_pos"]))
        self.redis_pipe.execute()

    def is_safe(self, pos_des, ori_des, linear_thresh = 0.2, ang_thresh=0.6):
        state = self.arm.get_state()
        if np.linalg.norm(pos_des - state["arm_pos"]) > linear_thresh:
            return False
        
        state_quat = state["arm_quat"]
        q0 = Quaternion(w=state_quat[3], x=state_quat[0], y=state_quat[1], z=state_quat[2])
        q1 = Quaternion(w=ori_des[3], x=ori_des[0], y=ori_des[1], z=ori_des[2])
        dist = Quaternion.absolute_distance(q0,q1)
        if dist > ang_thresh:
            return False
        
        return True

    def close(self):
        self.arm.close()


if __name__ == "__main__":
    env = HumanShadowKinovaEnv()
    env.reset()
    try:
        while True:
            env.step()
            time.sleep(POLICY_CONTROL_PERIOD) # not precise
    finally:
        env.close()
        print("Done kinova env")