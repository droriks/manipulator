from setuptools import find_packages, setup

package_name = 'manipulator_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dror',
    maintainer_email='drorks@terpmail.umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sine_wave_node = manipulator_trajectory.sine_wave_node:main',
            'playback_node = mainpulator_trajectory.playback_node:main'
        ],
    },
)
