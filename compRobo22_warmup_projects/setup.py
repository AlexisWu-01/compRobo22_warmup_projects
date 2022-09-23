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
    maintainer='Alexis',
    maintainer_email='xwu1@olin.edu',
    description='Warmup Project for CompRobo22',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_square = compRobo22_warmup_projects.drive_square:main',
            'wall_follower = compRobo22_warmup_projects.wall_follower:main',
            'person_follower = compRobo22_warmup_projects.person_follower:main',
            'obstacle_avoider = compRobo22_warmup_projects.obstacale_avoider:main',
            'teleop = compRobo22_warmup_projects.teleop:main',
            'finite_state_controller = compRobo22_warmup_projects.finite_state_controller:main'

            

        ],
    },
)
