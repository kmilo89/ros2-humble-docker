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

   ```sh
   ros2 pkg create name_package --build-type ament_python --dependencies rclpy
   ros2 pkg create --build-type ament_python <package_name>
   // Update package
   colcon build --symlink-install
   source ~/.bashrc 
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

3. **Create Python Node**

   Create a file `my_node.py` in the `my_package` directory:

   ```sh
   touch my_package/my_node.py
   chmod +x my_package/my_node.py
   ```

4. **Write node script**

   Open `my_package/my_node.py` and add the following:

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

5. **Modify `setup.py`**

   Make sure your `setup.py` looks like this to include your script:

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

6. **Compile the package **

   ```sh
   cd ~/ros2_ws
   colcon build
   ```

7. **Source of workspace**

   ```sh
   source install/setup.bash
   ```

8. **Run node**

   ```sh
   ros2 run my_package my_node
   ```

### **Additional Resources**

- **Official ROS 2 Documentation**: [https://docs.ros.org/en/foxy/index.html](https://docs.ros.org/en/humble/index.html)
- **ROS 2 Tutorials**: [https://index.ros.org/doc/ros2/Tutorials/](https://docs.ros.org/en/humble/Tutorials.html)
- **ROS 2 Forum**: [https://discourse.ros.org/](https://discourse.ros.org/)










### **Docker Commands**

docker exec -it ros_humble_container bash

docker run --rm -it --user ros --network=host --ipc=host -v $PWD/source:/source_code -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY ros-humble


distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
