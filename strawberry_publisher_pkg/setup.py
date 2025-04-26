from setuptools import setup

package_name = 'strawberry_publisher_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
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
