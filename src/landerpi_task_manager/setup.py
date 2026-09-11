from setuptools import find_packages, setup

package_name = 'landerpi_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='LanderPi Team',
    maintainer_email='team@example.com',
    description='Optional multi-goal task management for LanderPi.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_queue = landerpi_task_manager.task_queue:main',
        ],
    },
)
