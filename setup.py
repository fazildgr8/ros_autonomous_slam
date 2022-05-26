import os
from glob import glob
from setuptools import setup

package_name = 'ros_autonomous_slam'

setup(
 name=package_name,
 version='1.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools', ''],
 zip_safe=True,
 maintainer='fazildgr8',
 maintainer_email='mohamedfazilsulaiman@gmail.com',
 description='The ros package that implements RRT exploration on turtlebot3 Wffle Pi robot',
 license='TODO',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'filter = ros_autonomous_slam.scripts.filter:main',
             'assigner = ros_autonomous_slam.scripts.assigner:main'
     ],
   },
)