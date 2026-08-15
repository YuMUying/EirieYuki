from setuptools import find_packages, setup

package_name = "wcr_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="project-maintainers",
    maintainer_email="maintainers@example.com",
    description="Task-tree supervision and command fusion for the WCR robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fusion_supervisor = wcr_fusion.fusion_supervisor:main",
            "vision_camera_monitor = wcr_fusion.vision_camera_monitor:main",
            "mock_ultrasound_device = wcr_fusion.mock_ultrasound_device:main",
        ],
    },
)
