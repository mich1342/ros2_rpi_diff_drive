from setuptools import find_packages, setup

package_name = 'ros2_rpi_diff_drive'

zltech_drive = 'ros2_rpi_diff_drive/ZltechDrive'
odrive_drive = 'ros2_rpi_diff_drive/OdriveDrive'

diff_drive_kinematics = 'ros2_rpi_diff_drive/DiffDriveKinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, zltech_drive, odrive_drive, diff_drive_kinematics],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blibli',
    maintainer_email='michaeljonathan664@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_modbus = ros2_rpi_diff_drive.drive_modbus:main',
            'drive_odrive = ros2_rpi_diff_drive.drive_odrive:main'
        ],
    },
)
