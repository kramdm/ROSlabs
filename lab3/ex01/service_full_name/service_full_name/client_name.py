import sys

import rclpy
from rclpy.node import Node

from name_service.srv import SummFullName

class Client_name(Node):

    def __init__(self):
        super().__init__("Client_name_node")
        self.client_ = self.create_client(SummFullName, "sum_full_name")
        while not self.client_.wait_for_service(timeout_sec=1):
            self.get_logger().info("Waiting for server")

    def send_request(self, n1, n2, n3):
        request = SummFullName.Request()

        request.first_name = n1
        request.name = n2
        request.last_name = n3

        req_res = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, req_res)

        return req_res.result()
    
def main():
    rclpy.init()

    cn = Client_name()

    response = cn.send_request(*sys.argv[1:4])

    cn.get_logger().info(f"Operation result: {response.full_name}")

    cn.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
