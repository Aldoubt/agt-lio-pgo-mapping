from setuptools import find_packages, setup

setup(
    name='agt_greenhouse_annotation', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agt_greenhouse_annotation']),
        ('share/agt_greenhouse_annotation', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy', 'Shapely'],
    tests_require=['pytest'],
    zip_safe=False,
    maintainer='AGT Mapping Team', maintainer_email='xuanyang.robotics@gmail.com',
    description='Manual greenhouse topology annotation and provenance validator.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'greenhouse_annotator = agt_greenhouse_annotation.gui:main',
        'greenhouse_annotation_validate = agt_greenhouse_annotation.validator:main',
        'greenhouse_topology_export = agt_greenhouse_annotation.export:main',
        'greenhouse_annotation_freeze = agt_greenhouse_annotation.freeze:main',
    ]},
)
