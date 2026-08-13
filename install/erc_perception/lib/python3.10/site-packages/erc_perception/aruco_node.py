import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

COMP_IDS = set(range(51, 65))   # 51..64 inclusive

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.image_callback, 1)

        # Publish COMPRESSED output — much smaller/faster than raw
        self.pub = self.create_publisher(
            CompressedImage, '/erc/aruco/image/compressed', 1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info('Aruco node started (compressed output)...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            corners, ids = self.dedupe(corners, ids)
            comp_found = []
            for corner, mid in zip(corners, ids.flatten()):
                pts = corner.reshape(4, 2).astype(int)
                is_comp = int(mid) in COMP_IDS
                color = (0, 255, 0) if is_comp else (150, 150, 150)
                cv2.polylines(frame, [pts], True, color, 2)
                label = f'ID:{mid}' + (' *' if is_comp else '')
                cv2.putText(frame, label, tuple(pts[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if is_comp:
                    comp_found.append(int(mid))
            if comp_found:
                cv2.putText(frame, 'I SEE IT', (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                self.get_logger().info(f'Competition markers: {sorted(comp_found)}',
                                       throttle_duration_sec=1.0)

        # Encode as JPEG and publish as CompressedImage
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.pub.publish(out)

    def dedupe(self, corners, ids):
        best = {}
        for corner, mid in zip(corners, ids.flatten()):
            area = cv2.contourArea(corner.reshape(4, 2).astype(np.float32))
            mid = int(mid)
            if mid not in best or area > best[mid][0]:
                best[mid] = (area, corner)
        kept_ids = np.array([[k] for k in best], dtype=np.int32)
        kept_corners = [best[k][1] for k in best]
        return kept_corners, kept_ids

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()