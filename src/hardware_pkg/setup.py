from setuptools import find_packages, setup

package_name = 'hardware_pkg'

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
    maintainer='da',
    maintainer_email='da@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                        'gripper_node = hardware_pkg.gripper_node:main',
                        'robot2_gpio_gripper_node = hardware_pkg.robot2_gpio_gripper_node:main',

        ],
    },
)
