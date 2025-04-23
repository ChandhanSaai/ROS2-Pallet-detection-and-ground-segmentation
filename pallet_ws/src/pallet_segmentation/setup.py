from setuptools import find_packages, setup

package_name = 'pallet_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/inference_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chandhan',
    maintainer_email='chandhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference = pallet_segmentation.inference:main',
            'inference_segment = pallet_segmentation.inference_segment:main',
        ],
    },
)
