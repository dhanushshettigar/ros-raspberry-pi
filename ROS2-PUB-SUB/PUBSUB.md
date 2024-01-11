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

## Write the subscriber node

Return to ```ros2_ws/src/py_pubsub/py_pubsub``` to create the next node. 
Create New file named ```subscriber_member_function.py``` and copy bellow code

```bash
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Breaking the code

The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. The topic name and message type used by the publisher and subscriber must match to allow them to communicate.

```
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

The callback definition simply prints an info message to the console, along with the data it received. Recall that the publisher defines ```msg.data = 'Hello World: %d' % self.i```

```
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

The ```main``` definition is almost exactly the same, replacing the creation and spinning of the publisher with the subscriber.

Since this node has the same dependencies as the publisher, there’s nothing new to add to ```package.xml```. The ```setup.cfg``` file can also remain untouched.

## Add an entry point

Reopen ```setup.py``` and add the entry point for the subscriber node below the publisher’s entry point. The ```entry_points``` field should now look like this:

```bash
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

Make sure to save the file, and then your pub/sub system should be ready.

## Build and run

You likely already have the ```rclpy``` and ```std_msgs``` packages installed as part of your ROS 2 system.
 It’s good practice to run ```rosdep``` in the root of your workspace (ros2_ws) to check for missing dependencies before building:

```bash
rosdep install -i --from-path src --rosdistro iron -y
```

Still in the root of your workspace, ```ros2_ws```, build your new package:

```bash
colcon build --packages-select py_pubsub
```

Open a new terminal, navigate to ```ros2_ws```, and source the setup files:

```bash
source install/setup.bash
```
Now run the talker node:

```bash
ros2 run py_pubsub talker
```

The terminal should start publishing info messages every 0.5 seconds, like so:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node:

```bash
ros2 run py_pubsub listener
```

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```
Enter ```Ctrl+C``` in each terminal to stop the nodes from spinning.