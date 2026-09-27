from setuptools import setup

package_name = 'agt_map_localization_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT Mapping',
    maintainer_email='dev@agt.local',
    description='Isolated offline localization benchmark; no runtime registration code or publishers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agt_map_localization_benchmark = agt_map_localization_benchmark.cli:main',
        ],
    },
)
