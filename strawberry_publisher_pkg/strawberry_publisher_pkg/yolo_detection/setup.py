from setuptools import setup, find_packages

setup(
    name='yolo_detection',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'opencv-python',
        'numpy',
        'ultralytics',  # ultralytics 라이브러리 추가
        'tensorflow',   # tensorflow 라이브러리 추가
    ],
    zip_safe=True,
    author='lko',
    author_email='lko991111@gmail.com',
    description='YOLO 기반 딸기 검출 모듈',
    license='MIT',
)
