from setuptools import find_packages, setup

package_name = 'a24_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/closed_real_robot_setup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/open_real_robot_setup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/a24_setup.launch.py'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='the-hassan-shahzad',
    maintainer_email='hshahzad0277@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = a24_scripts.custom_odom_node:main',
            'pid_controller = a24_scripts.PID_controller:main',
            'tf_publisher = a24_scripts.tf_publisher:main',
            'open_loop_controller = a24_scripts.open_loop_controller:main'
        ],
    },
)
