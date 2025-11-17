from setuptools import find_packages, setup

package_name = "zh_gyak2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/zh_gyak2.launch.xml"]),
        ("share/" + package_name + "/rviz", ["rviz/zh_gyak2.rviz"]),
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
            "calc_displacement = zh_gyak2.calc_displacement:main",
            "calc_centroid = zh_gyak2.calc_centroid:main",
        ],
    },
)
