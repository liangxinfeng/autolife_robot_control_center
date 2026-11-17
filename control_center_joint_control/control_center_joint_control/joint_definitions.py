"""Robot joint names, UI labels and verified limits (all UI values are degrees)."""

import math


GROUPS = {
    'neck': {
        'label': '头颈',
        'command_key': 'neck_target_joints_position',
        'high_risk': False,
        'joints': [
            ('Joint_Neck_Roll', '颈部侧倾', -17.496227, 17.496227),
            ('Joint_Neck_Pitch', '颈部俯仰', -27.500070, 42.500105),
            ('Joint_Neck_Yaw', '颈部旋转', -57.660568, 57.660568),
        ],
    },
    'left_arm': {
        'label': '左臂',
        'command_key': 'left_arm_target_joints_position',
        'high_risk': False,
        'joints': [
            ('Joint_Left_Shoulder_Inner', '左肩内轴', -166.491226, 166.491226),
            ('Joint_Left_Shoulder_Outer', '左肩外轴', -9.500000, 178.408748),
            ('Joint_Left_UpperArm', '左上臂旋转', -165.500009, 165.500009),
            ('Joint_Left_Elbow', '左肘', -148.500351, -1.500000),
            ('Joint_Left_Forearm', '左前臂旋转', -167.499631, 167.499631),
            ('Joint_Left_Wrist_Upper', '左腕上轴', -78.502097, 78.502097),
            ('Joint_Left_Wrist_Lower', '左腕下轴', -81.498666, 81.498666),
        ],
    },
    'right_arm': {
        'label': '右臂',
        'command_key': 'right_arm_target_joints_position',
        'high_risk': False,
        'joints': [
            ('Joint_Right_Shoulder_Inner', '右肩内轴', -166.491226, 166.491226),
            ('Joint_Right_Shoulder_Outer', '右肩外轴', -178.500421, 9.500000),
            ('Joint_Right_UpperArm', '右上臂旋转', -165.459902, 165.459902),
            ('Joint_Right_Elbow', '右肘', -148.500351, -1.500000),
            ('Joint_Right_Forearm', '右前臂旋转', -167.499631, 167.499631),
            ('Joint_Right_Wrist_Upper', '右腕上轴', -78.484908, 78.484908),
            ('Joint_Right_Wrist_Lower', '右腕下轴', -81.498666, 81.498666),
        ],
    },
    'leg_waist': {
        'label': '腰腿（高风险）',
        'command_key': 'leg_waist_target_joints_position',
        'high_risk': True,
        'joints': [
            ('Joint_Ankle', '踝关节', -84.498289, 84.498289),
            ('Joint_Knee', '膝关节', -154.771562, 154.771562),
            ('Joint_Waist_Pitch', '腰部俯仰', -97.499847, 89.499910),
            ('Joint_Waist_Yaw', '腰部旋转', -179.408748, 179.408748),
        ],
    },
    'grippers': {
        'label': 'DM夹爪（原厂位置量）',
        'command_key': None,
        'high_risk': False,
        'joints': [
            ('Joint_Left_Gripper', '左夹爪', 10.0, 360.0),
            ('Joint_Right_Gripper', '右夹爪', 10.0, 360.0),
        ],
    },
}

FEEDBACK_KEYS = {
    'leg_waist': 'leg_waist_joint_state',
    'left_arm': 'left_arm_joint_state',
    'right_arm': 'right_arm_joint_state',
    'neck': 'neck_joint_state',
}

TARGET_FEEDBACK_KEYS = {
    'leg_waist': 'leg_waist_target_joint_state',
    'left_arm': 'left_arm_target_joint_state',
    'right_arm': 'right_arm_target_joint_state',
    'neck': 'neck_target_joint_state',
}

GRIPPER_FEEDBACK_KEYS = {
    'Joint_Left_Gripper': 'left_gripper_state',
    'Joint_Right_Gripper': 'right_gripper_state',
}

BASE_MODEL_JOINTS = [
    'Joint_Ground_Vehicle_X',
    'Joint_Ground_Vehicle_Y',
    'Joint_Ground_Vehicle_Z',
]

GROUP_ORDER = ['neck', 'left_arm', 'right_arm', 'leg_waist', 'grippers']

JOINT_TO_GROUP = {}
JOINT_LABELS = {}
JOINT_LIMITS = {}
for group_name, group in GROUPS.items():
    for name, label, lower, upper in group['joints']:
        JOINT_TO_GROUP[name] = group_name
        JOINT_LABELS[name] = label
        JOINT_LIMITS[name] = (float(lower), float(upper))

ALL_JOINTS = [
    joint[0]
    for group_name in GROUP_ORDER
    for joint in GROUPS[group_name]['joints']
]
BODY_JOINTS = [name for name in ALL_JOINTS if JOINT_TO_GROUP[name] != 'grippers']
GRIPPER_JOINTS = list(GRIPPER_FEEDBACK_KEYS)


def clamp_joint(name, value):
    """Return a finite, limit-clamped degree value."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} target is not finite')
    lower, upper = JOINT_LIMITS[name]
    return min(upper, max(lower, value))


def group_joint_names(group_name):
    return [item[0] for item in GROUPS[group_name]['joints']]


def step_toward(current, target, maximum_delta):
    """Move a scalar target no farther than maximum_delta from current."""
    current = float(current)
    target = float(target)
    maximum_delta = float(maximum_delta)
    if maximum_delta <= 0.0:
        raise ValueError('maximum_delta must be positive')
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + math.copysign(maximum_delta, delta)
