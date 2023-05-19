import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose
from collections import deque

MAX_DIFF = 0.1

class Pose(TPose):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        return f"(x={self.x:.1f}, theta={self.y:.1f})"
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF \
        and abs(self.y - other.y) < MAX_DIFF \


class Fila(deque):
    def __init__(self):
        super().__init__()
        new_pose = Pose()
        new_pose.x, new_pose.y = [[0.0,0.5]]
        self.enqueue(new_pose)
        new_pose.x, new_pose.y = [[0.5,0.0]]
        self.enqueue(new_pose)
        new_pose.x, new_pose.y = [[0.0,0.5]]
        self.enqueue(new_pose)
        new_pose.x, new_pose.y = [[0.5,0.0]]
        self.enqueue(new_pose)
        new_pose.x, new_pose.y = [[0.0,1.0]]
        self.enqueue(new_pose)
        new_pose.x, new_pose.y = [[1.0,0.0]]
        self.enqueue(new_pose)
        
    def enqueue(self, x):
        super().append(x)
    
    def dequeue(self):
        return super().popleft()

class Pilha(deque):
    def __init__(self, pilha):
        super().__init__(pilha)

    def append(self, valor):
        return super().append(valor)

    def pop(self):
        return super().pop()
    
class TurtleController(Node):
    def __init__(self, fila, control_period=0.02):
        super().__init__('turtle_controller')
        self.pose = Pose(x=0.0)
        self.setpoint = Pose(x=0.0)
        self.fila = fila
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )
        self.control_timer = self.create_timer(
                timer_period_sec=control_period,
                callback=self.control_callback
        )

    def control_callback(self):
        msg = Twist()
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.5 if y_diff > 0 else -0.5
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        else:
            msg.linear.x = 0.0
        self.publisher.publish(msg)
        
    def update_setpoint(self):
            self.setpoint = self.pose + self.fila.dequeue()

    def pose_callback(self, msg):
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == 0.0:
            self.update_setpoint()


def main(args=None):
    rclpy.init(args=args)
    mc = Fila()
    tc = TurtleController(mc)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()