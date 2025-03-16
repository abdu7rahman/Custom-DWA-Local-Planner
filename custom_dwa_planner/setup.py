from setuptools import find_packages, setup

package_name = 'custom_dwa_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Ensure resource folder exists
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdulrahman',
    maintainer_email='abdulrahmantrm@gmail.com',
    description='A custom Dynamic Window Approach (DWA) planner for ROS2 TurtleBot3',
    license='MIT',  # Change this if necessary
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = custom_dwa_planner.dwa_planner:main',
        ],
    },
)
