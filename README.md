# ros2-humble-docker

### **Setup your Workspace **

1. **Create a Workspace**

   ```sh
   cd /ros2_ws
   colcon build
   ```

2. **Source Workspace**

   ```sh
   source /ros2_ws/install/setup.bash
   ```

### **First ROS2 Node**

We will to create a simple package and a python node.

1. **Create a new package**

   ```sh
   cd /ros2_ws/src
   ros2 pkg create --build-type ament_python my_package
   cd my_package
   ```

2. **Structure of package**

   The package structure should look like this::

   ```
   my_package/
   ├── package.xml
   ├── setup.cfg
   ├── setup.py
   └── my_package
       └── __init__.py
   ```






3. **Crear un nodo Python**

   Crea un archivo llamado `my_node.py` en el directorio `my_package`:

   ```sh
   touch my_package/my_node.py
   chmod +x my_package/my_node.py
   ```

4. **Escribe el código del nodo**

   Abre `my_package/my_node.py` y agrega lo siguiente:

   ```python
   import rclpy
   from rclpy.node import Node

   class MyNode(Node):
       def __init__(self):
           super().__init__('my_node')
           self.get_logger().info('Hello ROS 2')

   def main(args=None):
       rclpy.init(args=args)
       node = MyNode()
       rclpy.spin(node)
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

5. **Modifica `setup.py`**

   Asegúrate de que tu `setup.py` se vea así para incluir tu script:

   ```python
   from setuptools import setup

   package_name = 'my_package'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='TODO: Package description',
       license='TODO: License declaration',
       entry_points={
           'console_scripts': [
               'my_node = my_package.my_node:main',
           ],
       },
   )
   ```

6. **Compilar el paquete**

   ```sh
   cd ~/ros2_ws
   colcon build
   ```

7. **Fuente el espacio de trabajo**

   ```sh
   source install/setup.bash
   ```

8. **Ejecuta el nodo**

   ```sh
   ros2 run my_package my_node
   ```

### 4. **Recursos Adicionales**

- **Documentación Oficial de ROS 2**: [https://docs.ros.org/en/foxy/index.html](https://docs.ros.org/en/humble/index.html)
- **Tutoriales de ROS 2**: [https://index.ros.org/doc/ros2/Tutorials/](https://docs.ros.org/en/humble/Tutorials.html)
- **Foro ROS**: [https://discourse.ros.org/](https://discourse.ros.org/)
- **Cursos en línea**: Plataformas como Coursera, Udemy y edX tienen cursos de ROS y ROS 2.

### 5. **Próximos Pasos**

Una vez que estés cómodo con la creación de nodos simples, puedes explorar:

- **Publicadores y suscriptores**: Aprende a enviar y recibir mensajes.
- **Servicios y acciones**: Implementa comunicaciones más complejas.
- **TF2**: Maneja transformaciones entre diferentes sistemas de coordenadas.
- **SLAM y Navegación**: Para robots móviles.
- **Integración con sensores y actuadores**: Para trabajar con hardware real.

ROS 2 es una plataforma poderosa para el desarrollo de aplicaciones robóticas. A medida que avances, podrás construir sistemas más complejos y personalizar ROS 2 según tus necesidades. ¡Buena suerte en tu aprendizaje!


# Commands

docker exec -it ros_humble_container bash

docker run --rm -it --user ros --network=host --ipc=host -v $PWD/source:/source_code -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY ros-humble


distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker



Create Workspace ROS2

sudo apt-get update
mkdir -p ros2_ws/src
cd ros2_ws
colcon build 

Create package
ros2 pkg create name_package --build-type ament_python --dependencies rclpy
ros2 pkg create --build-type ament_python <package_name>
 - in pkg folder
entry_points={
        'console_scripts': [
            "test_node = name_package.name_node:main"
        ],

colcon build 
source ~/.bashrc 

Update package
colcon build --symlink-install
source ~/.bashrc 