import time
import sys
import IPython
e = IPython.embed

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, MY_MASTER_START_ARM_POS, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import get_arm_joint_positions, torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions

def prep_robots(master_bot, puppet_bot):
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot.dxl.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit
    torque_on(puppet_bot)
    torque_on(master_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    start_arm_qpos_master = MY_MASTER_START_ARM_POS[:6]
    #move_arms([master_bot, puppet_bot], [start_arm_qpos] * 2, move_time=1)
    move_arms([master_bot, puppet_bot], [start_arm_qpos_master, start_arm_qpos], move_time=1)
    # move grippers to starting position
    #move_grippers([master_bot, puppet_bot], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)
    #初始化位置
    #move_grippers([master_bot, puppet_bot], [0.3, -2.3], move_time=2)
    move_grippers([master_bot, puppet_bot], [0.72, -1.638], move_time=2)


def press_to_start(master_bot):
    # press gripper to start data collection
    # disable torque for only gripper joint of master robot to allow user movement
    master_bot.dxl.robot_torque_enable("single", "gripper", False)

    print(f'Close the gripper to start')
    #close_thresh = -0.3
    close_thresh = 0.0
    pressed = False
    while not pressed:
        gripper_pos = get_arm_gripper_positions(master_bot)
        #gripper_join = get_arm_joint_positions(master_bot)
        #print(gripper_join)
        print(gripper_pos)
        if abs(gripper_pos - close_thresh) < 0.01:
            pressed = True
            kay = 1
        time.sleep(DT/10)
    torque_off(master_bot)
    print(f'Started!')

def press_to_puppet_start(puppet_bot):
    #puppet_bot.dxl.robot_torque_enable("single", "gripper", False)
    pressed = False
    while not pressed:
        gripper_pos = get_arm_gripper_positions(puppet_bot)
        #gripper_join = get_arm_joint_positions(master_bot)
        #print(gripper_join)
        print(gripper_pos)


def teleop(robot_side):
    """ A standalone function for experimenting with teleoperation. No data recording. """
    puppet_bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_{robot_side}', init_node=True)
    master_bot = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_{robot_side}', init_node=False)

    prep_robots(master_bot, puppet_bot)
    #press_to_puppet_start(puppet_bot)
    press_to_start(master_bot)

    ### Teleoperation loop
    gripper_command = JointSingleCommand(name="gripper")
    while True:
        # sync joint positions
        master_state_joints = master_bot.dxl.joint_states.position[:6]
        puppet_bot.arm.set_joint_positions(master_state_joints, blocking=False)
        # sync gripper positions
        master_gripper_joint = master_bot.dxl.joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint)
        gripper_command.cmd = puppet_gripper_joint_target
        puppet_bot.gripper.core.pub_single.publish(gripper_command)

        gripper_pos1 = get_arm_gripper_positions(puppet_bot)
        gripper_pos2 = get_arm_gripper_positions(master_bot)
        print(gripper_pos1)
        print(gripper_pos2)

        # sleep DT
        time.sleep(DT)


if __name__=='__main__':
    side = sys.argv[1]
    teleop(side)