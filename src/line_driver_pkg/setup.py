from setuptools import find_packages, setup

package_name = 'line_driver_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', 
            ['launch/line_driver.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fizzer',
    maintainer_email='miti@phas.ubc.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "circle_driver = line_driver_pkg.circle_driver:main",
            "line_driver = line_driver_pkg.line_driver:main",
        ],
    },
)
