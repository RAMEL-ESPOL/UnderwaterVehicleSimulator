from setuptools import find_packages, setup

package_name = 'auv_max_graphics'

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
    maintainer='Iesus Davila',
    maintainer_email='iesusdavila@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_mov_lineal = auv_max_graphics.scripts.pid_mov_lineal:main',
            'pid_mov_angular = auv_max_graphics.scripts.pid_mov_angular:main',
        ],
    },
)
