import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'joint_state_publisher'

setup(
    name=package_name,
    version='2.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all test files into the correct share directory
        ('share/' + package_name + '/test', glob.glob('test/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='David V. Lu!!',
    author_email='davidvlu@gmail.com',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A python node to publish `sensor_msgs/JointState` messages for a '
        'robot described with URDF.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher.joint_state_publisher:main',
        ],
    },
)
