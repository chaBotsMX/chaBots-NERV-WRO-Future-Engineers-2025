from setuptools import setup

package_name = 'otos_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/otos_reader/launch',['launch/otos_reader.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_correo@example.com',
    description='Nodo ROS 2 en Python para leer el SparkFun OTOS con bias y reconexión',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'otos_node = otos_reader.otos_node:main',
        ],
    },
)
