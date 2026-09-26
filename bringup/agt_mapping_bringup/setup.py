from glob import glob
from setuptools import setup

package_name = 'agt_mapping_bringup'

setup(name=package_name, version='0.3.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
], install_requires=['setuptools'], zip_safe=True, maintainer='AGT Mapping Team',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='Verified offline/live mapping workflow and pipeline composition.', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mapping_wait_ready = agt_mapping_bringup.session_runtime:wait_ready_main',
        'mapping_export_verified = agt_mapping_bringup.session_runtime:export_main',
        'mapping_live_supervisor = agt_mapping_bringup.live_supervisor:supervisor_main',
        'mapping_review = agt_mapping_bringup.review:main',
        'mapping_map_release = agt_mapping_bringup.map_release:main',
    ]})
