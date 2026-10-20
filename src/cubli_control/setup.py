from glob import glob

from setuptools import find_packages, setup

package_name = 'cubli_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryuzuu',
    maintainer_email='htrang.cyb@gmail.com',
    description='Simulator, state estimator, ESP32 hardware driver and jump-up/LQR controller for a 1D Cubli.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = cubli_control.sim_node:main',
            'controller_node = cubli_control.controller_node:main',
            'estimator_node = cubli_control.estimator_node:main',
            'hw_driver_node = cubli_control.hw_driver_node:main',
            'fake_esp32 = cubli_control.fake_esp32:main',
        ],
    },
)
