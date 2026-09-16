from glob import glob
from setuptools import setup

package_name = 'agt_map_editor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT Mapping Team',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='RViz interactive marker editor for map refinement rules.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'map_refinement_editor = agt_map_editor.editor_node:main',
    ]},
)