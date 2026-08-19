# 文件作用：定义可视化 Python ROS 2 包的安装信息和节点入口点。
from setuptools import find_packages, setup


package_name = "cleanbot_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cleanbot Engineering",
    maintainer_email="engineering@cleanbot.local",
    description="Optional RViz visualization adapters for cleanbot.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "cleanbot_visualization_node = "
            "cleanbot_visualization.visualization_node:main",
        ],
    },
)
