from glob import glob
from setuptools import setup

package_name = 'agt_coverage_authoring'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/examples', glob('examples/*')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'shapely>=2,<3', 'PyYAML', 'Pillow'],
    extras_require={'reference': ['rsplan==1.0.10'], 'test': ['pytest>=8,<9']},
    maintainer='AGT Mapping Team', maintainer_email='xuanyang.robotics@gmail.com',
    description='Offline greenhouse coverage authoring and existing Route asset adaptation',
    license='Apache-2.0',
    entry_points={'console_scripts': ['agt_coverage_authoring = agt_coverage_authoring.cli:main']},
)
