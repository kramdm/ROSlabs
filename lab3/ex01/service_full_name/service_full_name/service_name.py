import rclpy
from rclpy.node import Node

from name_service.srv import SummFullName

class Service_name(Node):

    def __init__(self):
        super().__init__("Service_name_node")
        self.srv_ = self.create_service(SummFullName, "sum_full_name", self.callback)
    
    def callback(self, request, response):
        response.full_name = f"{request.first_name} {request.name} {request.last_name}"
        self.get_logger().info(f"Have request: '{request.first_name} {request.name} {request.last_name}'")
        return response

def main():
    rclpy.init()

    sn = Service_name()

    rclpy.spin(sn)

    sn.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
