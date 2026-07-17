from setuptools import find_packages, setup

package_name = "ipe_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Haoming",
    maintainer_email="124701939+haomingyi@users.noreply.github.com",
    description="Application-layer references and commands for IPE joint control",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "reference_manager = ipe_control.reference_manager:main",
            "send_joint_goal = ipe_control.send_joint_goal:main",
        ],
    },
)
