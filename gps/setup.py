from setuptools import find_packages, setup

package_name = 'gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrm',
    maintainer_email='mrm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps=gps.gpsNode:main',
            ],
    },
)
