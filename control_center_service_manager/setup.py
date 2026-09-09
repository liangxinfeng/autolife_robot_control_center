from setuptools import find_packages, setup


package_name = 'control_center_service_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Lightweight user service manager for the Autolife control center.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'service_manager = control_center_service_manager.panel:main',
            'joint_reset_once = control_center_service_manager.joint_reset_once:main',
        ],
    },
)
