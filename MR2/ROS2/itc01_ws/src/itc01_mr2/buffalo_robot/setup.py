from setuptools import setup
import os
from glob import glob
package_name = 'buffalo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsagx-6',
    maintainer_email='jsagx-6@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps4 = buffalo_robot.ps4_control:main',
            'ps4v2 = buffalo_robot.ps4_controlV2:main',
            'omni_teleop = buffalo_robot.teleopkeybord_omni:main',
            'buffalov1 = buffalo_robot.mecanum_pidV1:main',
            'buffalov2 = buffalo_robot.mecanum_pidV2:main',
            'buffalo_sim = buffalo_robot.pid_simulation:main',
            'omni_pidBlue = buffalo_robot.omni_pidV6_Blue:main',
            'omni_pidRed = buffalo_robot.omni_pidV6_RED:main',
            'Omni_model = buffalo_robot.Omni_model:main',
            'can_blue = buffalo_robot.can_buffalo_Blue:main',
            'can_red = buffalo_robot.can_buffalo_Red:main',
            'Omni_PidSim = buffalo_robot.Omni_PidSim:main',
            'omni_mpcv1 = buffalo_robot.omni_mpcV1:main',
            'omni_mpcv2 = buffalo_robot.omni_mpcV2:main',
            'omni_mpcsim = buffalo_robot.omni_mpcSimV1:main',
            'imu = buffalo_robot.imu:main',
            'imu_ha9 = buffalo_robot.imu_HFI_a9:main',
            'imu_cal = buffalo_robot.imu_calibrate:main',
            'cal_goal = buffalo_robot.Calculate_position_Red:main',
        ],
    },
)
