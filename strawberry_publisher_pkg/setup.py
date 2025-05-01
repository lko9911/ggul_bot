# sudo apt install python3-pip
# cd /ros2_ws2/ggul_bot/strawberry_publisher_pkg/strawberry_publisher_pkg/
# pip3 install -r requirements.txt

from setuptools import setup, find_packages

package_name = 'strawberry_publisher_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'opencv-python',
    'ultralytics',
    'tensorflow',
    'numpy==1.23.5',  
    ],
    zip_safe=True,
    maintainer='lko',
    maintainer_email='lko991111@gmail.com',
    description='딸기 위치 퍼블리셔',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strawberry_publisher = strawberry_publisher_pkg.strawberry_publisher:main',
        ],
    },
)
