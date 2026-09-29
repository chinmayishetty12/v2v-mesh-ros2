import socket
import struct
import threading
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

class V2VMeshNode(Node):
    def __init__(self):
        super().__init__('v2v_mesh_node')
        
        # Declare vehicle ID parameter (default to Vehicle #4 for ego)
        self.declare_parameter('vehicle_id', 4)
        self.vehicle_id = self.get_parameter('vehicle_id').value
        
        # UDP Network Setup for local V2V mesh simulation
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT = 5005
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.UDP_IP, self.UDP_PORT))
        self.socket.setblocking(False)

        # Simulated ground truth states of neighboring mining fleet vehicles
        self.fleet_states = {
            1: {'x': 0.0, 'y': -18.0, 'vx': 0.0, 'vy': 0.45, 'heading': 1.57},
            2: {'x': 18.0, 'y': 2.0, 'vx': 0.0, 'vy': 0.50, 'heading': 3.14},
            3: {'x': -7.5, 'y': -8.0, 'vx': 0.0, 'vy': 0.22, 'heading': 1.57},
            5: {'x': 0.0, 'y': 22.0, 'vx': 0.0, 'vy': -0.55, 'heading': -1.57},
            6: {'x': -11.5, 'y': 14.0, 'vx': 0.0, 'vy': -0.48, 'heading': -1.57}
        }

        # ROS 2 Publisher for neighboring V2V data
        self.v2v_pub = self.create_publisher(String, '/v2v/neighbor_vehicles', 10)

        # Timers: 10 Hz broadcast loop and background receive thread
        self.create_timer(0.1, self.broadcast_telemetry)
        
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(f"V2V Mesh Node Active for Vehicle #{self.vehicle_id}")

    def broadcast_telemetry(self):
        ego_state = self.fleet_states.get(self.vehicle_id, {'x': -7.5, 'y': -18.0, 'vx': 0.0, 'vy': 0.58, 'heading': 1.57})
        
        # Pack message payload: id, x, y, vx, vy, heading, timestamp
        data = struct.pack('ifffffd', 
                           self.vehicle_id, 
                           ego_state['x'], 
                           ego_state['y'], 
                           ego_state['vx'], 
                           ego_state['vy'], 
                           ego_state['heading'], 
                           time.time())
        try:
            self.socket.sendto(data, (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            self.get_logger().error(f"Broadcast failed: {e}")

    def receive_loop(self):
        while rclpy.ok():
            try:
                packet, addr = self.socket.recvfrom(1024)
                
                # Simulate 2% packet drop rate
                if random.random() < 0.02:
                    continue
                
                # Simulate artificial network/processing latency (~18ms)
                time.sleep(0.018)

                unpacked = struct.unpack('ifffffd', packet)
                remote_id = unpacked[0]
                
                # Skip own broadcast
                if remote_id == self.vehicle_id:
                    continue
                
                rx, ry, rvx, rvy, rhead, rtime = unpacked[1:]
                
                # Coordinate Transformation: Absolute to Relative wrt Ego Vehicle (base_link)
                ego_state = self.fleet_states.get(self.vehicle_id, {'x': -7.5, 'y': -18.0, 'heading': 1.57})
                dx = rx - ego_state['x']
                dy = ry - ego_state['y']
                
                cos_h = math.cos(-ego_state['heading'])
                sin_h = math.sin(-ego_state['heading'])
                rel_x = dx * cos_h - dy * sin_h
                rel_y = dx * sin_h + dy * cos_h

                # Publish relative position info to ROS 2 topic
                output_msg = String()
                output_msg.data = f"ID:{remote_id} | Rel_X:{rel_x:.2f}m | Rel_Y:{rel_y:.2f}m"
                self.v2v_pub.publish(output_msg)

            except BlockingIOError:
                time.sleep(0.005)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = V2VMeshNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
