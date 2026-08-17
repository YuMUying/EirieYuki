from glob import glob
import os

from setuptools import find_packages, setup


package_name = "wcr_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="project-maintainers",
    maintainer_email="maintainers@example.com",
    description="Low-latency RGB-D weld localization for top and probe cameras.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgbd_weld_localizer = wcr_vision.rgbd_weld_localizer:main",
        ],
    },
)
