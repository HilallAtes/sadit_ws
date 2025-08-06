import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
from std_msgs.msg import Bool

class QrCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/sadit/camera2_rgb/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for the boolean detection status
        self.qr_found_publisher = self.create_publisher(
            Bool,
            'qr_code_found',
            10
        )
        
        self.cv_bridge = CvBridge()
        self.get_logger().info('QR Code Detector Node has been started.')

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            decoded_objects = decode(cv_image)
            zoom_factor = 2.0
            zoomed_image = cv2.resize(cv_image, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_LINEAR)

            decoded_objects = decode(zoomed_image)
                      
            qr_found_msg = Bool()
            qr_found_msg.data = False
            
            if decoded_objects:
                qr_found_msg.data = True
                
                for obj in decoded_objects:
                    # Get bounding box coordinates and draw a rectangle
                    (x, y, w, h) = obj.rect
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Optionally, you can also print the decoded data
                    data = obj.data.decode('utf-8')
                    self.get_logger().info(f"QR Code Found: {data}")

            # Display the image in a new window
            cv2.imshow("QR Code Detection", cv_image)
            cv2.waitKey(1)
            
            # Publish the boolean status
            self.qr_found_publisher.publish(qr_found_msg)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    qr_code_node = QrCodeDetector()
    rclpy.spin(qr_code_node)
    
    # Clean up
    qr_code_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()