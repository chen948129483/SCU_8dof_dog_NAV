import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

rclpy.init()
node = Node('test_status_pub')
pub = node.create_publisher(Bool, '/dog_status/is_stepping', 10)

msg = Bool()
msg.data = False

print("开始测试：每隔 2 秒发送一次 False，不发送 True")

try:
    while rclpy.ok():
        pub.publish(msg)
        print("==== 发送 FALSE ====")
        time.sleep(2.0)

except KeyboardInterrupt:
    pass

finally:
    node.destroy_node()
    rclpy.shutdown()