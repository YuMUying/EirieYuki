from setuptools import find_packages, setup


package_name = "wcr_probe_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="project-maintainers",
    maintainer_email="maintainers@example.com",
    description="Vision-guided transverse rail control for the ultrasonic probe.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "probe_rail_controller = wcr_probe_control.probe_rail_controller:main",
            "seam_tracking_manager = wcr_probe_control.seam_tracking_manager:main",
            "mock_linear_motor_driver = wcr_probe_control.mock_linear_motor_driver:main",
        ],
    },
)
