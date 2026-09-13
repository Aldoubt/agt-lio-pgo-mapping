from setuptools import setup

package_name = 'agt_mapping_artifacts'

setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
], install_requires=['setuptools'], zip_safe=True, maintainer='AGT Mapping Team',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='Schema, provenance and integrity boundary for mapping artifacts.', license='Apache-2.0')
