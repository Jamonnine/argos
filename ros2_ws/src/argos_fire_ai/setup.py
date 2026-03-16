from setuptools import find_packages, setup

package_name = 'argos_fire_ai'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamonnine',
    maintainer_email='jamonnine@github.com',
    description='ARGOS AI fire/smoke detection (AGPL-3.0)',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [],
    },
)
