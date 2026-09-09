"""Resolve and initialize the source-backed pose/action storage directory."""

import os
from pathlib import Path
import shutil


LIBRARY_DIRECTORY_NAME = 'motion_library'
ACTION_FILE_NAME = 'saved_joint_actions.yaml'
POSE_FILE_NAME = 'saved_joint_poses.yaml'
ACTION_PICKLE_DIRECTORY_NAME = 'action_pkls'


def resolve_motion_library_dir(package_share):
    override = (
        os.environ.get('CONTROL_CENTER_JOINT_CONTROL_LIBRARY_DIR', '')
        or os.environ.get('WHOLE_BODY_JOINT_CONTROL_LIBRARY_DIR', '')
    ).strip()
    if override:
        return Path(override).expanduser().resolve()

    package_share = Path(package_share).resolve()
    for parent in package_share.parents:
        for source_package in (
            parent / 'src' / 'control_center_joint_control',
            parent / 'src' / 'autolife_robot_control_center'
            / 'control_center_joint_control',
        ):
            if (source_package / 'package.xml').is_file():
                return (source_package / LIBRARY_DIRECTORY_NAME).resolve()
    return (package_share / LIBRARY_DIRECTORY_NAME).resolve()


def initialize_motion_library(directory, legacy_config_directory):
    directory = Path(directory).resolve()
    legacy = Path(legacy_config_directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / ACTION_PICKLE_DIRECTORY_NAME).mkdir(
        parents=True, exist_ok=True
    )
    defaults = {
        ACTION_FILE_NAME: 'version: 1\nactions: {}\n',
        POSE_FILE_NAME: 'version: 1\nposes: {}\n',
    }
    for filename, empty_document in defaults.items():
        destination = directory / filename
        if destination.exists():
            continue
        source = legacy / filename
        if source.is_file():
            shutil.copy2(source, destination)
        else:
            destination.write_text(empty_document, encoding='utf-8')
    return directory


def action_pickle_path(directory, action_name):
    # Action validation already forbids slashes and control characters.
    return (
        Path(directory).resolve()
        / ACTION_PICKLE_DIRECTORY_NAME
        / f'{str(action_name).strip()}.pkl'
    )
