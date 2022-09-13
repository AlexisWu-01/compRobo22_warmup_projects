from setuptools import setup

package_name = 'compRobo22_warmup_projects'

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
    maintainer='phillip',
    maintainer_email='phillip.post@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_square = compRobo22_warmup_projects.drive_square:main'
            # 'subscriber = warmup_projects.subscriber:main'
        ],
    },
)