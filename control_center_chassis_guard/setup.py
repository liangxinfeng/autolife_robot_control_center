from setuptools import find_packages, setup

package_name = 'control_center_chassis_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # The teleop smoothing limits are read from the file next to velocity_ramp.py,
    # so they have to ship with the package as well.
    package_data={package_name: ['*.yaml']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Chassis control ownership lease for the control center manual driving pages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
