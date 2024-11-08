from setuptools import setup

package_name = 'object_detection_trainer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BalazsSzonyi',
    maintainer_email='@gmail.com',
    description='object_detection_trainer_node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_trainer_node = object_detection_trainer.object_detection_trainer_node:main'
        ],
    },
)