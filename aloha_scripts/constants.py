### Task parameters
#DATA_DIR = '<put your data dir here>'

DATA_DIR = '/root/interbotix_ws/src/aloha/aloha_scripts/data'
TASK_CONFIGS = {
    'aloha_wear_shoe':{
        'dataset_dir': DATA_DIR + '/aloha_wear_shoe',
        'num_episodes': 50,
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
    'operation_test':{
        'dataset_dir': DATA_DIR + '/operation_test',
        'num_episodes': 45,
        'episode_len': 1400,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
    'exchange_cube':{
        'dataset_dir': DATA_DIR + '/exchange_cube',
        'num_episodes': 50,
        'episode_len': 700,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
        'put_cube':{
        'dataset_dir': DATA_DIR + '/put_cube',
        'num_episodes': 50,
        'episode_len': 1100,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
        'red_cube':{
        'dataset_dir': DATA_DIR + '/red_cube',
        'num_episodes': 50,
        'episode_len': 800,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
        'blue_cube':{
        'dataset_dir': DATA_DIR + '/blue_cube',
        'num_episodes': 50,
        'episode_len': 800,
        'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    },
        'sim_insertion_human': {
        'dataset_dir': DATA_DIR + '/sim_insertion_human',
        'num_episodes': 50,
        'episode_len': 500,
        'camera_names': ['top']
    },
}

### ALOHA fixed constants
DT = 0.02
JOINT_NAMES = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
START_ARM_POSE = [0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239, 
0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239]

MY_MASTER_START_ARM_POS = [0, -0.96, 1.16, 0, -0.3, 0.0, 0.02239, 0.3]

# note: absolute path
# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
MASTER_GRIPPER_POSITION_OPEN = 0.7041
MASTER_GRIPPER_POSITION_CLOSE = -0.0598
PUPPET_GRIPPER_POSITION_OPEN = -1.4572
PUPPET_GRIPPER_POSITION_CLOSE = -2.261 

# Gripper joint limits (qpos[6])
MASTER_GRIPPER_JOINT_OPEN = 0.72
MASTER_GRIPPER_JOINT_CLOSE = 0.1000
PUPPET_GRIPPER_JOINT_OPEN = -1.6380
PUPPET_GRIPPER_JOINT_CLOSE = -2.400


############################ Helper functions ############################

#将夹持器的位置值归一化到一个标准化的范围内
MASTER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - MASTER_GRIPPER_POSITION_CLOSE) / (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE)
PUPPET_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - PUPPET_GRIPPER_POSITION_CLOSE) / (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE)
#将已经归一化的夹持器位置值反归一化到实际的位置范围内
MASTER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE) + MASTER_GRIPPER_POSITION_CLOSE
PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE) + PUPPET_GRIPPER_POSITION_CLOSE
#将主控制器（Master）的位置值映射到从机（Puppet）的位置值
MASTER2PUPPET_POSITION_FN = lambda x: PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN(MASTER_GRIPPER_POSITION_NORMALIZE_FN(x))

#将夹持器关节位置归一化到一个标准化的范围内
MASTER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE)
#将给定的夹持器关节位置值归一化到特定范围内
PUPPET_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE)
#将已经归一化的夹持器关节位置值反归一化到实际的位置范围内
MASTER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE
#将归一化的关节位置值反转回原始范围内的值
PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) + PUPPET_GRIPPER_JOINT_CLOSE
#将主控制器（Master）的夹持器关节位置映射到从机Puppet）的夹持器关节位置
MASTER2PUPPET_JOINT_FN = lambda x: PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(MASTER_GRIPPER_JOINT_NORMALIZE_FN(x))

#将夹持器的速度值归一化到一个标准化的范围内
MASTER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE)
#将给定的夹持器速度值归一化到特定范围内
PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE)

#将主控制器（Master）的位置值转换为夹持器关节位置值
MASTER_POS2JOINT = lambda x: MASTER_GRIPPER_POSITION_NORMALIZE_FN(x) * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE
#将夹持器的关节位置值转换为主控制器的位置值。
MASTER_JOINT2POS = lambda x: MASTER_GRIPPER_POSITION_UNNORMALIZE_FN((x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE))

#根据这个参数计算出一个对应的关节位置
PUPPET_POS2JOINT = lambda x: PUPPET_GRIPPER_POSITION_NORMALIZE_FN(x) * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) + PUPPET_GRIPPER_JOINT_CLOSE
#将给定的关节位置 x 转换为对应的夹持器位置
PUPPET_JOINT2POS = lambda x: PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN((x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE))

#求解两个值的平均数  1/2 （一个夹持器的开启位置或角度 + 夹持器的关闭位置或角度）
MASTER_GRIPPER_JOINT_MID = (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE)/2