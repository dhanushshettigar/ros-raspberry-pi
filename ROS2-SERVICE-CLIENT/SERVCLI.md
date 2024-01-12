#  Simple Service and Client

When _nodes_ communicate using _services_, the node that sends a request for data is called the **client node**, and the one that responds to the request is the **service node**. The structure of the request and response is determined by a ```.srv``` file.

The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

## Create Package

Navigate into ```ros2_ws/src``` and create a new package:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
```

Your terminal will return a message verifying the creation of your package ```py_srvcli``` and all its necessary files and folders.

The ```--dependencies``` argument will automatically add the necessary dependency lines to ```package.xml```. ```example_interfaces``` is the package that includes the ```.srv``` file you will need to structure your requests and responses:

```
int64 a
int64 b
---
int64 sum
```

The first two lines are the parameters of the request, and below the dashes is the response.

## Write the service node

Inside the ```ros2_ws/src/py_srvcli/py_srvcli``` directory, create a new file called ```service_member_function.py``` and paste the following code within:

```bash
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Write the client node

Inside the ```ros2_ws/src/py_srvcli/py_srvcli``` directory, create a new file called ```client_member_function.py``` and paste the following code within:

```bash
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Add an entry point

The ```entry_points``` field of your ```setup.py``` file should look like this:

```bash
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

## Build and run

It’s good practice to run ```rosdep``` in the root of your workspace (```ros2_ws```) to check for missing dependencies before building:

```bash
rosdep install -i --from-path src --rosdistro iron -y
```

Navigate back to the root of your workspace, ```ros2_ws```, and build your new package:

```bash
colcon build --packages-select py_srvcli
```

Open a new terminal, navigate to ```ros2_ws```, and source the setup files:

```bash
source install/setup.bash
```

Now run the service node:

```bash
ros2 run py_srvcli service
```

The node will wait for the client’s request.

Open another terminal and source the setup files from inside ros2_ws again. Start the client node, followed by any two integers separated by a space:

```bash
ros2 run py_srvcli client 2 3
```

If you chose ```2``` and ```3```, for example, the client would receive a response like this:

```bash
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```
Return to the terminal where your service node is running. You will see that it published log messages when it received the request:

```bash
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

Enter ```Ctrl+C``` in the server terminal to stop the node from spinning.