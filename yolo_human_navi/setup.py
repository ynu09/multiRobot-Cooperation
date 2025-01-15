from setuptools import find_packages, setup

package_name = 'yolo_human_navi'

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
    maintainer='ynu',
    maintainer_email='ynu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detector = yolo_human_navi.human_detector:main',
            'turtlebot_navi = yolo_human_navi.turtlebot_navi:main'
        ],
    },
)
