from setuptools import find_packages, setup

package_name = 'landerpi_sandbox_display'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi Team',
    maintainer_email='team@example.com',
    description='Sandbox map and robot-state display.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sandbox_display_node = landerpi_sandbox_display.sandbox_display_node:main',
        ],
    },
)

