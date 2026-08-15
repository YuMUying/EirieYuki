from setuptools import find_packages, setup


package_name = "wcr_sensor_sync"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="project-maintainers",
    maintainer_email="maintainers@example.com",
    description="Timestamp interpolation for INS, odometry and probe rail data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inspection_context_server = wcr_sensor_sync.inspection_context_server:main",
        ],
    },
)
