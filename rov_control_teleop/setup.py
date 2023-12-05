from setuptools import find_packages, setup

package_name = 'rov_control_teleop'

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
    maintainer='rov-robot',
    maintainer_email='iesusdavila@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = rov_control_teleop.scripts.rov_teleop:main',
            'graficar_pid_long = rov_control_teleop.scripts.graficar_pid_long:main',
            'graficar_pid_angular = rov_control_teleop.scripts.graficar_pid_angular:main',
        ],
    },
)
