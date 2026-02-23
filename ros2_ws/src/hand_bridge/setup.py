from setuptools import setup

package_name = "hand_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="move_hand",
    maintainer_email="dev@example.com",
    description="Minimal ROS2 bridge for Linker Hand PyBullet simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_driver_node = hand_bridge.sim_driver_node:main",
        ],
    },
)
