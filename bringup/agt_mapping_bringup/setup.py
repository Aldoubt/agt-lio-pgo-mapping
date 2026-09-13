from glob import glob
from setuptools import setup

package_name = 'agt_mapping_bringup'

setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
], install_requires=['setuptools'], zip_safe=True, maintainer='AGT Mapping Team',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='Composition launch boundary for AGT mapping framework pipelines.', license='Apache-2.0')
