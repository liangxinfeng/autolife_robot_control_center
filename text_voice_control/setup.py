from glob import glob
from setuptools import find_packages, setup

package_name = "text_voice_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_ZH.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Text command bridge for controlling existing robot TTS output.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "text_voice_control_node = text_voice_control.text_voice_control_node:main",
            "asr_tts_bridge_node = text_voice_control.asr_tts_bridge_node:main",
            "say_text = text_voice_control.say_text:main",
        ],
    },
)
