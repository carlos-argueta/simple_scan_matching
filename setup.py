from setuptools import setup

package_name = 'simple_scan_matching'

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
    maintainer='carlos',
    maintainer_email='carlos.argueta@soulhackerslabs.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_matcher_laser = simple_scan_matching.scan_matcher_laser:main',
            'scan_matcher_pc = simple_scan_matching.scan_matcher_pc:main'
        ],
    },
)
