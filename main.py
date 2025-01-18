# Author: Jimmy Wu
# Date: October 2024

import argparse
import time
import numpy as np
import redis
from itertools import count
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeWriter
from policies import TeleopPolicy, RemotePolicy
from redis_utils import encode_matlab, decode_matlab

TIDYBOT_NUC_HOST = "172.24.69.38"
TIDYBOT_PORT = 6379
TIDYBOT_PWD = ""

KEY_CONTROL_POS = "tidybot::control_pos"
KEY_CONTROL_ORI = "tidybot::control_ori"
KEY_CONTROL_GRIPPER = "tidybot::control_gripper"

KEY_SENSOR_POS = "tidybot::sensor_pos"
KEY_SENSOR_ORI = "tidybot::sensor_ori"
KEY_SENSOR_GRIPPER = "tidybot::sensor_gripper"

KEY_TELEOP_STATUS = "tidybot::teleop_status"

def should_save_episode(writer):
    if len(writer) == 0:
        print('Discarding empty episode')
        return False

    # Prompt user whether to save episode
    while True:
        user_input = input('Save episode (y/n)? ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('Discarding episode')
            return False
        print('Invalid response')

def run_episode(env, policy, writer=None, redis_pipe=None):
    # Reset the env
    print('Resetting env...')
    env.reset()
    obs = env.get_obs()
    redis_pipe.set(KEY_TELEOP_STATUS, encode_matlab(np.zeros(1)))
    redis_pipe.set(KEY_CONTROL_POS, encode_matlab(obs["arm_pos"]))
    redis_pipe.set(KEY_CONTROL_ORI, encode_matlab(obs["arm_quat"]))
    redis_pipe.set(KEY_CONTROL_GRIPPER, encode_matlab(obs["gripper_pos"]))
    redis_pipe.execute()
    print('Env has been reset')

    # Wait for user to press "Start episode"
    print('Press "Start episode" in the web app when ready to start new episode')
    policy.reset()
    print('Starting new episode')
    redis_pipe.set(KEY_TELEOP_STATUS, encode_matlab(np.ones(1)))
    redis_pipe.execute()

    episode_ended = False
    start_time = time.time()
    for step_idx in count():
        # Enforce desired control freq
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # Get latest observation
        obs = env.get_obs()

        # Get action
        action = policy.step(obs)

        # No action if teleop not enabled
        if action is None:
            continue

        # Execute valid action on robot
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # Record executed action
                writer.step(obs, action)
            if not episode_ended:
                redis_pipe.set(KEY_CONTROL_POS, encode_matlab(action["arm_pos"]))
                redis_pipe.set(KEY_CONTROL_ORI, encode_matlab(action["arm_quat"]))
                redis_pipe.set(KEY_CONTROL_GRIPPER, encode_matlab(action["gripper_pos"]))
                redis_pipe.set(KEY_SENSOR_POS, encode_matlab(obs["arm_pos"]))
                redis_pipe.set(KEY_SENSOR_ORI, encode_matlab(obs["arm_quat"]))
                redis_pipe.set(KEY_SENSOR_GRIPPER, encode_matlab(obs["gripper_pos"]))
                redis_pipe.execute()

        # Episode ended
        elif not episode_ended and action == 'end_episode':
            episode_ended = True
            redis_pipe.set(KEY_TELEOP_STATUS, encode_matlab(np.zeros(1)))
            redis_pipe.execute()
            print('Episode ended')

            if writer is not None and should_save_episode(writer):
                # Save to disk in background thread
                writer.flush_async()

            print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # Ready for env reset
        elif action == 'reset_env':
            break

    if writer is not None:
        # Wait for writer to finish saving to disk
        writer.wait_for_flush()

def main(args):
    # Create env
    if args.sim:
        from mujoco_env import MujocoEnv
        if args.teleop:
            env = MujocoEnv(show_images=True)
        else:
            env = MujocoEnv()
    else:
        from real_env import RealEnv
        env = RealEnv()

    # Create policy
    if args.teleop:
        # Initialize redis
        _redis = redis.Redis(
            host=TIDYBOT_NUC_HOST, port=TIDYBOT_PORT, password=TIDYBOT_PWD
        )
        redis_pipe = _redis.pipeline()
        policy = TeleopPolicy()
    else:
        redis_pipe = None
        policy = RemotePolicy()

    try:
        while True:
            writer = EpisodeWriter(args.output_dir) if args.save else None
            run_episode(env, policy, writer, redis_pipe)
    finally:
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--teleop', action='store_true')
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--output-dir', default='data/demos')
    main(parser.parse_args())
