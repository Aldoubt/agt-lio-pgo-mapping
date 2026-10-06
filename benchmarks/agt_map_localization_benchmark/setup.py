from setuptools import setup

package_name = 'agt_map_localization_benchmark'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'GREENHOUSE.md']),
        ('share/' + package_name + '/config', ['config/greenhouse_scenes.example.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AGT Mapping',
    maintainer_email='dev@agt.local',
    description='Isolated offline localization benchmark; no runtime registration code or publishers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agt_map_localization_benchmark = agt_map_localization_benchmark.cli:main',
            'agt_greenhouse_relocalization_benchmark = agt_map_localization_benchmark.greenhouse:main',
            'agt_plot_greenhouse_basin = agt_map_localization_benchmark.plot_greenhouse:main',
            'topk_ambiguity_analysis = agt_map_localization_benchmark.topk_ambiguity_analysis:main',
            'topk_trace_replay = agt_map_localization_benchmark.topk_trace_replay:main',
            'agt_run_relocalization_query = agt_map_localization_benchmark.relocalization_mvp:main',
            'agt_verify_relocalization_evidence = agt_map_localization_benchmark.relocalization_mvp:verify_main',
            'agt_export_relocalization_candidate = agt_map_localization_benchmark.relocalization_mvp:candidate_overlay_main',
        ],
    },
)
