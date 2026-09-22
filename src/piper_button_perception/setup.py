from glob import glob
import os
from setuptools import find_packages, setup


package_name = "piper_button_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zlf",
    maintainer_email="zlf@todo.todo",
    description="Colored elevator button perception tools for Piper.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generate_button_dataset = piper_button_perception.dataset_generator:main",
            "preview_button_dataset = piper_button_perception.preview_dataset:main",
            "button_target_node = piper_button_perception.button_target_node:main",
        ],
    },
)
