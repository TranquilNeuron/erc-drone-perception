import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import traceback

FX, FY = 797.7728881835938, 797.6380004882812
CX, CY = 620.0264282226562, 330.038330078125

MARKER_SIZE_M = 0.10          # <-- your marker's black-square side, in meters

DISP_OFFSET = 0.039     # inverse-depth correction (1/m), from calibration table           # fill in after the distance table


PATCH = 5

class DepthCheck(Node):
    def __init__(self):
        super().__init__('depth_check')
        self.bridge = CvBridge()
        self.depth = None
        self.depth_encoding = None

        self.create_subscription(Image, '/oak/rgb/image_raw', self.rgb_cb, 1)
        self.create_subscription(Image, '/oak/stereo/image_raw', self.depth_cb, 1)
        self.pub = self.create_publisher(
            CompressedImage, '/erc/depth_check/compressed', 1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters())
        self.K = np.array([[FX,0,CX],[0,FY,CY],[0,0,1]], dtype=np.float64)
        self.dist = np.zeros(5)
        self.get_logger().info('Depth check started.')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_encoding = msg.encoding

    def sample_depth_m(self, u, v):
        """Patch-median RAW depth in meters (uncorrected)."""
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        u, v = int(u), int(v)
        if not (0 <= u < w and 0 <= v < h):
            return None
        patch = self.depth[max(0,v-PATCH):v+PATCH+1,
                           max(0,u-PATCH):u+PATCH+1].astype(np.float32)
        vals = patch[patch > 0]
        if vals.size == 0:
            return None
        return float(np.median(vals)) / 1000.0

    def correct(self, Z):
        """Apply inverse-depth calibration correction."""
        inv = 1.0 / Z + DISP_OFFSET
        return 1.0 / inv if inv > 0 else None

    def rgb_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- DETECT FIRST, on the clean image (before any drawing) ---
        try:
            corners, ids, _ = self.detector.detectMarkers(frame)
        except Exception as e:
            self.get_logger().error(f'detect error: {e}')
            corners, ids = None, None

        # --- NOW draw the crosshair + HUD (safe, detection already done) ---
        cx, cy = int(round(CX)), int(round(CY))
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (255,0,255), 1)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (255,0,255), 1)
        cv2.circle(frame, (cx, cy), 3, (255,0,255), -1)

        try:
            n = 0 if ids is None else len(ids)
            cv2.putText(frame, f'markers={n}  depth={"OK" if self.depth is not None else "NONE"}',
                        (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            if n == 0:
                cv2.putText(frame, 'NO MARKER IN VIEW', (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            else:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                c = corners[0].reshape(4, 2).astype(np.float64)
                u, v = float(c[:,0].mean()), float(c[:,1].mean())
                du, dv = u - CX, v - CY

                cv2.circle(frame, (int(u), int(v)), 4, (0,255,0), -1)
                cv2.line(frame, (cx, cy), (int(u), int(v)), (0,255,255), 1)

                Zraw = self.sample_depth_m(u, v)
                if Zraw is not None:
                    Zc = self.correct(Zraw)
                    X = (u - CX) * Zc / FX
                    Y = (v - CY) * Zc / FY
                    stereo_txt  = f'RAW Z={Zraw:.3f}   CORR Z={Zc:.3f}'
                    xyz_txt     = f'X={X:+.3f} Y={Y:+.3f} Z={Zc:.3f}'
                else:
                    Zc = None
                    stereo_txt = 'STEREO Z: INVALID (no depth at pixel)'
                    xyz_txt = ''

                s = MARKER_SIZE_M / 2.0
                obj = np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]], np.float64)
                ok, rvec, tvec = cv2.solvePnP(obj, c, self.K, self.dist)
                Zpnp = float(tvec[2]) if ok else None

                cv2.putText(frame, stereo_txt, (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                if xyz_txt:
                    cv2.putText(frame, xyz_txt, (20, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 2)

                self.get_logger().info(
                    f'CALIB rawZ={Zraw} corrZ={Zc} pnpZ={Zpnp} '
                    f'offset_px=({du:+.0f},{dv:+.0f})',
                    throttle_duration_sec=0.5)
        except Exception as e:
            self.get_logger().error(f'draw error: {e}\n{traceback.format_exc()}')

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DepthCheck())
    rclpy.shutdown()

if __name__ == '__main__':
    main()