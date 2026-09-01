from glob import glob
from setuptools import find_packages, setup

package_name = 'landerpi_base_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi Team',
    maintainer_email='team@example.com',
    description='Simulation and hardware adapter for the LanderPi base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_interface_node = landerpi_base_driver.sim_interface_node:main',
            'cmd_vel_adapter_node = landerpi_base_driver.cmd_vel_adapter_node:main',
        ],
    },
)

