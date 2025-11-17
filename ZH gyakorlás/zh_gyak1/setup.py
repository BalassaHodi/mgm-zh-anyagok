from setuptools import find_packages, setup

package_name = "zh_gyak1"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/zh_gyak1.launch.xml"]),
        ("share/" + package_name + "/rviz", ["rviz/zh_gyak1.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="balassa",
    maintainer_email="hodi.balassa@edu.bme.hu",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "save_path = zh_gyak1.save_path:main",
            "pub_path = zh_gyak1.pub_path:main",
        ],
    },
)
