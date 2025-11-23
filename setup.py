import os
from glob import glob
from setuptools import setup

package_name = 'webots_ros2_turtlebot4'
package_dir = os.path.join(os.getcwd(), package_name)

data_files = [('share/ament_index/resource_index/packages',['resource/' + package_name]), ('share/' + package_name, ['package.xml']),]

data_files.append(('share/' + package_name + '/protos', glob('protos/*.proto')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*.wbt')))
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name + '/launch',glob('launch/*.py')))

# copy also the mesh files
data_files.append(('share/' + package_name + '/meshes/turtlebot4', [
    'meshes/turtlebot4/camera_bracket.dae',
    'meshes/turtlebot4/oakd_lite.dae',
    'meshes/turtlebot4/oakd_pro.dae',
    'meshes/turtlebot4/rplidar.dae',
    'meshes/turtlebot4/shell_collision.dae',
    'meshes/turtlebot4/shell.dae',
    'meshes/turtlebot4/tower_sensor_plate.dae',
    'meshes/turtlebot4/tower_standoff.dae',
    'meshes/turtlebot4/tower.dae',
    'meshes/turtlebot4/weight_block.dae'
]))

data_files.append(('share/' + package_name + '/meshes/create3', [
    'meshes/create3/body_visual.dae',
    'meshes/create3/bumper_collision.dae',
    'meshes/create3/bumper_visual.dae'
]))

packages = [package_name]

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Turtlebot4 Webots ROS2 bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_intensity_bridge = webots_ros2_turtlebot4.irIntensityBridge:main',
            'cliff_intensity_bridge = webots_ros2_turtlebot4.cliffIntensityBridge:main',
        ],
    },
)