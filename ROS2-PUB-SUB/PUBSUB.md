#  Simple Publisher and Subscriber 

In this task, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

## Create a Package

Packages should be created in the ```src``` directory, not the root of the workspace. So, navigate into ```ros2_ws/src```, and run the package creation command:

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

Your terminal will return a message verifying the creation of your package py_pubsub and all its necessary files and folders.

## Write the publisher node

Navigate into ```ros2_ws/src/py_pubsub/py_pubsub```

Create New file named ```publisher_member_function.py```

Copy the bellow code

```bash
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Breaking the code 

The first lines of code after the comments import ```rclpy``` so its ```Node``` class can be used.

```
import rclpy
from rclpy.node import Node
```
The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.

```
from std_msgs.msg import String
```

These lines represent the node’s dependencies.

Next, the MinimalPublisher class is created, which inherits from (or is a subclass of) Node.

```
class MinimalPublisher(Node):
```
Following is the definition of the class’s constructor. ```super().__init__``` calls the Node class’s constructor and gives it your node name, in this case ```minimal_publisher```.

```create_publisher``` declares that the node publishes messages of type String (imported from the ```std_msgs.msg module```), over a topic named ```topic```, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

Next, a timer is created with a callback to execute every 0.5 seconds. ```self.i``` is a counter used in the callback.

```
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```
```timer_callback``` creates a message with the counter value appended, and publishes it to the console with ```get_logger().info```.

```
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

Lastly, the main function is defined.

```
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

```
First the ```rclpy``` library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.

## Add dependencies

Navigate one level back to the ```ros2_ws/src/py_pubsub``` directory, where the ```setup.py```, ```setup.cfg```, and ```package.xml``` files have been created for you.

Open ```package.xml``` with your text editor.

Make sure to fill in the <description>, <maintainer> and <license> tags:

```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

After the lines above, add the following dependencies corresponding to your node’s import statements:

```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

This declares the package needs ```rclpy``` and ```std_msgs``` when its code is executed.

Make sure to save the file.

# Add an entry point

Open the setup.py file. Again, match the maintainer, maintainer_email, description and license fields to your package.xml:

```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

Add the following line within the ```console_scripts``` brackets of the ```entry_points``` field:

```bash
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},

```

Don’t forget to save.