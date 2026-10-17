from setuptools import setup, find_packages

package_name = 'bridge_imu'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),               # 패키지 인덱스 마커 파일 설치
        ('share/' + package_name, ['package.xml']),      # package.xml 설치
        ('share/' + package_name + '/launch', ['launch/bridge_imu_launch.py']),  # 예: launch 파일 설치
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='[email protected]',
    description='...패키지 설명...',
    license='...License...',
    entry_points={
        'console_scripts': [
            'imu_bridge= bridge_imu.imu_bridge:main',
        ],
    },
)

