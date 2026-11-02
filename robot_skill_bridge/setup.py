from setuptools import find_packages, setup

package_name = 'robot_skill_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pepper_skills.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML', 'Pillow'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='pepper-project',
    maintainer_email='dev@example.invalid',
    description='Puente genérico IA -> habilidades -> acciones ROS 2',
    license='MIT',
    entry_points={'console_scripts': ['skill_bridge = robot_skill_bridge.main:main']},
)
