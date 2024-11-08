from setuptools import setup

package_name = 'path_planner_trainer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BalazsSzonyi',
    maintainer_email='@gmail.com',
    description='path_planner_trainer_node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_trainer_node = path_planner_trainer.path_planner_trainer_node:main'
        ],
    },
)