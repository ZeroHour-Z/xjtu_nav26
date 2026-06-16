from setuptools import setup, find_packages

package_name = 'rm_decision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config/trees', [
            'config/trees/nav.yaml',
            'config/trees/RMUC.yaml',
            'config/trees/RMUL.yaml',
            'config/trees/test.yaml',
        ]),
        (f'share/{package_name}/launch', [
            'launch/bt.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='xjtu',
    maintainer_email='dev@xjtu.local',
    description='Behavior Tree framework for ROS 2 (Humble) using py_trees, with YAML/Python trees and Nav2 action integration.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bt_node = rm_decision.tree_node:main',
            'bt_render = rm_decision.loader:render_main',
        ],
    },
) 
