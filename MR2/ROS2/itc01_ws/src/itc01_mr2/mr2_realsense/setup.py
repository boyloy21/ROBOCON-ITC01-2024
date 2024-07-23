from setuptools import setup

package_name = 'mr2_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
          'yolov9_blue = mr2_realsense.yolov9_rs_blue:main',
          'yolov9_red = mr2_realsense.yolov9_rs_red:main',
        ],
    },
)
