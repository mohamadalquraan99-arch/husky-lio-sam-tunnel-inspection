from setuptools import find_packages, setup


package_name = 'tunnel_mission_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamad Alquraan',
    maintainer_email='mohamad@example.com',
    description='Tunnel baseline and inspection mission orchestration.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_manager = tunnel_mission_manager.mission_manager:main',
        ],
    },
)
