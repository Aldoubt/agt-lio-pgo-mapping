from glob import glob
from setuptools import setup

package_name = 'agt_mapping_exporter'

setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
], install_requires=['setuptools'], zip_safe=True, maintainer='AGT Mapping Team',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='Mapping artifact export orchestration and validation boundary.', license='Apache-2.0',
    entry_points={'console_scripts': ['mapping_artifact_exporter = agt_mapping_exporter.exporter_node:main']})
