from glob import glob
from setuptools import find_packages, setup


package_name = 'control_center_joint_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (
            'share/' + package_name,
            ['package.xml', 'README.md', 'AI_MIGRATION_GUIDE.md'],
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        (
            'share/' + package_name + '/motion_library',
            glob('motion_library/*.yaml'),
        ),
        (
            'share/' + package_name + '/motion_library/action_pkls',
            glob('motion_library/action_pkls/*.pkl'),
        ),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes/robot_v2_2', glob('meshes/robot_v2_2/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Whole-body joint monitoring, embedded 3D preview and safe control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_bridge = control_center_joint_control.joint_bridge:main',
            'joint_control_panel = control_center_joint_control.control_panel:main',
        ],
    },
)
