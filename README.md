# ros2-humble-docker

¡Genial! Aprender ROS 2 (Robot Operating System 2) es una excelente elección si estás interesado en la robótica y en desarrollar aplicaciones robóticas avanzadas. Aquí te dejo una guía paso a paso para comenzar con ROS 2:

### 1. **Instalación de ROS 2**

Primero, necesitas instalar ROS 2 en tu sistema. Aquí hay una guía básica para instalar ROS 2 Humble Hawksbill en Ubuntu 22.04:

#### Instalación de ROS 2 Humble en Ubuntu 22.04

1. **Configura las fuentes de Debian**

   ```sh
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   ```

2. **Instala ROS 2**

   ```sh
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. **Fuente el setup script**

   ```sh
   source /opt/ros/humble/setup.bash
   ```

4. **Instala dependencias adicionales**

   ```sh
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-argcomplete
   ```

### 2. **Configura tu entorno de trabajo**

1. **Crea un espacio de trabajo**

   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```

2. **Fuente el espacio de trabajo**

   ```sh
   source ~/ros2_ws/install/setup.bash
   ```

3. **Agregar al .bashrc (opcional)**

   Para no tener que ejecutar `source` cada vez que abras una nueva terminal, puedes agregarlo a tu `.bashrc`:

   ```sh
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### 3. **Primer Nodo en ROS 2**

Vamos a crear un paquete simple y un nodo en Python.

1. **Crear un nuevo paquete**

   ```sh
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_package
   cd my_package
   ```

2. **Estructura del paquete**

   La estructura de tu paquete debería verse así:

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
