from setuptools import find_packages, setup

package_name = 'landerpi_error_evaluator'

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
    description='Arrival decision and navigation error evaluation.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)

