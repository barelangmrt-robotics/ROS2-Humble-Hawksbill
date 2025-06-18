from setuptools import find_packages, setup

package_name = 'ros2_python_standard_msg'

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
    maintainer='rusaven',
    maintainer_email='adityagading265@gmail.com',
    description='Tutoial ROS2 Python dengan standard message',
    license='Rusaven',

    # daftarkan source file
    entry_points={
        'console_scripts': [
            'pub = ros2_python_standard_msg.pub:main', #nama_file = nama_package.nama_file:main
            'sub = ros2_python_standard_msg.sub:main',
        ],
    },
)
