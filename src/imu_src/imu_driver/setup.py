from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
        ('share/' + package_name + '/launch', ['launch/dual_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaushik',
    maintainer_email='kaushik@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={'test':['pytest']},
    entry_points={
        'console_scripts': [
            'driver = imu_driver.driver:main',
        ],
    },
)
