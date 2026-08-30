from setuptools import find_packages, setup

package_name = 'agibot_x2_pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='cate10aprile@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_arm = agibot_x2_pkg_py.move_arm:main',
            'gripper_controller = agibot_x2_pkg_py.gripper_controller:main',
            'pick_place_teleop = agibot_x2_pkg_py.pick_place_teleop:main',
        ],
    },
)
