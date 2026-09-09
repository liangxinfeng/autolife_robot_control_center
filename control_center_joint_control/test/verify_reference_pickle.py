"""Compare a generated action export against shake_hand_example.pkl."""

import json
from pathlib import Path
import pickle
import pickletools
import sys
import tempfile

from control_center_joint_control.action_library import load_action_library
from control_center_joint_control.pose_library import load_pose_library
from control_center_joint_control.trajectory_pickle import export_action_pickle


FORBIDDEN_OPCODES = {
    'GLOBAL', 'STACK_GLOBAL', 'REDUCE', 'NEWOBJ', 'NEWOBJ_EX', 'BUILD',
    'OBJ', 'INST', 'EXT1', 'EXT2', 'EXT4', 'PERSID', 'BINPERSID',
}


def signature(path):
    raw = Path(path).read_bytes()
    operations = sorted({
        operation.name
        for operation, _argument, _position in pickletools.genops(raw)
    })
    forbidden = FORBIDDEN_OPCODES.intersection(operations)
    if forbidden:
        raise ValueError(f'unsafe pickle operations: {sorted(forbidden)}')
    data = pickle.loads(raw)
    keys = list(json.loads(data[0][1]))
    return (
        type(data).__name__,
        type(data[0]).__name__,
        [type(value).__name__ for value in data[0]],
        operations,
        keys,
    )


def main():
    root = Path(sys.argv[1]).resolve()
    library = root / 'motion_library'
    actions = load_action_library(library / 'saved_joint_actions.yaml')
    poses = load_pose_library(library / 'saved_joint_poses.yaml')
    name = next(iter(actions))
    baseline = poses.get('默认', next(iter(poses.values())))['joints_deg']
    output = Path(tempfile.gettempdir()) / 'whole_body_export_compat_test.pkl'
    try:
        result = export_action_pickle(output, actions[name], baseline)
        reference = signature(root / 'shake_hand_example.pkl')
        generated = signature(output)
        assert reference[:4] == generated[:4]
        assert reference[4] == generated[4]
        print(
            f'PASS: {name} exported as {result["sample_count"]} samples, '
            f'{result["duration_sec"]:.3f}s, {output.stat().st_size} bytes; '
            'pickle types/opcodes/JSON keys match reference'
        )
    finally:
        output.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
