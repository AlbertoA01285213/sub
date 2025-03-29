from setuptools import setup

package_name = 'sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['pid'],  # si no está en un paquete, solo el script
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto',
    maintainer_email='tuemail@example.com',
    description='Nodo PID en ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid = sub.pid:main',
        ],
    },
)