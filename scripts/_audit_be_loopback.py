#!/usr/bin/env python3
"""Decisive test: does BEST_EFFORT -> BEST_EFFORT deliver over Cyclone on loopback
(multicast disabled)? Two SEPARATE processes (real DDS), sensor-data QoS both ends."""
import time
import multiprocessing as mp


def pub_proc():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    n = Node("be_pub_test")
    p = n.create_publisher(Image, "/audit_be_test", qos_profile_sensor_data)
    m = Image()
    m.height, m.width, m.encoding, m.step, m.data = 1, 1, "mono8", 1, bytes([0])
    n.create_timer(0.2, lambda: p.publish(m))
    end = time.time() + 8
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


def sub_proc(q):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    n = Node("be_sub_test")
    c = {"c": 0}
    n.create_subscription(Image, "/audit_be_test",
                          lambda msg: c.__setitem__("c", c["c"] + 1),
                          qos_profile_sensor_data)
    end = time.time() + 8
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    q.put(c["c"])
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    q = mp.Queue()
    ps = mp.Process(target=sub_proc, args=(q,))
    pp = mp.Process(target=pub_proc)
    ps.start()
    time.sleep(1.0)
    pp.start()
    pp.join()
    ps.join()
    print(f"BEST_EFFORT->BEST_EFFORT loopback: sub received {q.get()} of ~35 msgs")
