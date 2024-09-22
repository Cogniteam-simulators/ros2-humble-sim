from setuptools import setup

package_name = 'ros2_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch', ['launch/bringup_launch.py','launch/localization_launch.py', 'launch/navigation_launch.py']),
        ('share/'+ package_name , ['resource/nav2_params.yaml']), 
        ('share/'+ package_name +'/maps/' , ['resource/map/map.yaml']),  
        ('share/'+ package_name +'/maps/', ['resource/map/map.pgm']),  



    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yakir',
    maintainer_email='yakirhuri21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_robot_sim_node = ros2_robot_sim.ros2_robot_sim_node:main'
        ],
    },
)
