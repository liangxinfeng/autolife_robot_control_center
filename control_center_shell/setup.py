from glob import glob
from setuptools import find_packages, setup


package_name = 'control_center_shell'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Integrated robot control center shell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_center = control_center_shell.app:main',
        ],
    },
)
