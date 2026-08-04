import os, glob
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

DEBUG = False       # True -> reject boxes, per-candidate metrics, mask topic

# --- intrinsics + depth calibration (720p; re-read camera_info if you change res) ---
FX, FY = 797.7728881835938, 797.6380004882812
CX, CY = 620.0264282226562, 330.038330078125
DISP_OFFSET = 0.039

# --- color ---
LOWER = np.array([35, 50, 40]); UPPER = np.array([70, 255, 255])

# --- morphology ---
OPEN_SIZE, CLOSE_SIZE = 3, 25

# --- detection filters ---
MIN_AREA, MIN_ASPECT = 100, 2.5
SOLIDITY_MIN, EXTENT_MAX = 0.70, 0.90
USE_HU, HU_MAX = True, 2.5
LEN_MIN, LEN_MAX = 0.10, 0.30
WID_MIN, WID_MAX = 0.003, 0.06
PATCH = 5
TEMPLATE_GLOB = os.path.expanduser('~/erc_ws/probe_template*.npy')

# --- tracking ---
MAX_MISSES, MATCH_DIST, MAX_TRACKS = 15, 80, 6


def make_csrt():
    if hasattr(cv2, 'TrackerCSRT_create'):
        return cv2.TrackerCSRT_create()
    return cv2.legacy.TrackerCSRT_create()


class Track:
    def __init__(self, tid, frame, bbox):
        self.id = tid
        self.tracker = make_csrt(); self.tracker.init(frame, bbox)
        self.bbox = bbox; self.misses = 0; self.pose = None; self.alive = True

    def refresh(self, frame, bbox, pose):
        self.tracker = make_csrt(); self.tracker.init(frame, bbox)
        self.bbox = bbox; self.pose = pose; self.misses = 0

    def center(self):
        x, y, w, h = self.bbox
        return (x + w / 2.0, y + h / 2.0)


class ProbeNode(Node):
    def __init__(self):
        super().__init__('probe_node')
        self.bridge = CvBridge()
        self.depth = None
        self.tracks = []
        self.next_id = 0

        self.create_subscription(Image, '/oak/rgb/image_raw',    self.rgb_cb,   1)
        self.create_subscription(Image, '/oak/stereo/image_raw', self.depth_cb, 1)
        self.img_pub  = self.create_publisher(CompressedImage, '/erc/probe/image/compressed', 1)
        self.pose_pub = self.create_publisher(PoseArray, '/erc/probe/poses', 1)
        if DEBUG:
            self.mask_pub = self.create_publisher(CompressedImage, '/erc/probe/mask/compressed', 1)

        self.open_k  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_SIZE, OPEN_SIZE))
        self.close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_SIZE, CLOSE_SIZE))

        self.templates = []
        if USE_HU:
            for f in sorted(glob.glob(TEMPLATE_GLOB)):
                try: self.templates.append(np.load(f, allow_pickle=True))
                except Exception as e: self.get_logger().warn(f'template {f}: {e}')
            self.get_logger().info(f'Loaded {len(self.templates)} template(s).')
        self.get_logger().info('Probe node ready.')

    # ---------- depth ----------
    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _correct(self, Zraw):
        inv = 1.0 / Zraw + DISP_OFFSET
        return 1.0 / inv if inv > 0 else None

    def depth_masked(self, contour):
        if self.depth is None: return None
        m = np.zeros(self.depth.shape[:2], np.uint8)
        cv2.drawContours(m, [contour], -1, 255, -1)
        d = self.depth[(m == 255) & (self.depth > 0)].astype(np.float32)
        return self._correct(float(np.median(d)) / 1000.0) if d.size else None

    def depth_point(self, u, v):
        if self.depth is None: return None
        h, w = self.depth.shape[:2]; u, v = int(u), int(v)
        if not (0 <= u < w and 0 <= v < h): return None
        p = self.depth[max(0,v-PATCH):v+PATCH+1, max(0,u-PATCH):u+PATCH+1].astype(np.float32)
        vals = p[p > 0]
        return self._correct(float(np.median(vals)) / 1000.0) if vals.size else None

    def hu_distance(self, mask0, sil):
        if not self.templates: return 0.0
        region = cv2.bitwise_and(mask0, sil)
        rc, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not rc: return 999.0
        cc = max(rc, key=cv2.contourArea)
        return min(cv2.matchShapes(cc, t, cv2.CONTOURS_MATCH_I1, 0.0) for t in self.templates)

    # ---------- detection ----------
    def detect(self, frame, mask0, mask):
        dets = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area <= MIN_AREA: continue
            (rcx, rcy), (rw, rh), ang = cv2.minAreaRect(c)
            length_px, width_px = max(rw, rh), min(rw, rh)
            aspect = length_px / width_px if width_px > 0 else 0.0
            extent = area / (length_px * width_px) if length_px * width_px > 0 else 0.0
            sil = np.zeros(mask0.shape, np.uint8); cv2.drawContours(sil, [c], -1, 255, -1)
            sil_area = cv2.countNonZero(sil)
            fill = cv2.countNonZero(cv2.bitwise_and(mask0, sil)) / sil_area if sil_area else 0.0
            hu = self.hu_distance(mask0, sil) if USE_HU else 0.0
            u, v = int(rcx), int(rcy)

            reason = None
            if   aspect < MIN_ASPECT:    reason = 'shape'
            elif fill   < SOLIDITY_MIN:  reason = 'hollow'
            elif extent > EXTENT_MAX:    reason = 'rect'
            elif USE_HU and hu > HU_MAX: reason = 'hu'
            if reason:
                if DEBUG:
                    box = cv2.boxPoints(((rcx,rcy),(rw,rh),ang)).astype(int)
                    cv2.drawContours(frame, [box], 0, (0,0,255), 1)
                    cv2.putText(frame, f'{reason}', (u-30, v-12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                continue

            Z = self.depth_masked(c)
            if Z is None:
                if DEBUG: cv2.putText(frame,'no depth',(u-30,v-12),
                                      cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1)
                continue
            real_len = length_px * Z / FX
            real_wid = width_px  * Z / FX
            if not (LEN_MIN <= real_len <= LEN_MAX and WID_MIN <= real_wid <= WID_MAX):
                if DEBUG: cv2.putText(frame,f'size {real_len*1000:.0f}',(u-30,v-12),
                                      cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,140,255),1)
                continue

            X = (u - CX) * Z / FX; Y = (v - CY) * Z / FY
            bx, by, bw, bh = cv2.boundingRect(c)
            dets.append({'bbox': (int(bx),int(by),int(bw),int(bh)),
                         'center': (float(u), float(v)),
                         'xyz': (float(X), float(Y), float(Z))})
        return dets

    # ---------- tracking ----------
    def track(self, frame, dets):
        for t in self.tracks:
            ok, bb = t.tracker.update(frame)
            if ok: t.bbox = tuple(int(i) for i in bb)
            else:  t.alive = False
        matched = set()
        for det in dets:
            du_, dv_ = det['center']; best, bd = None, MATCH_DIST
            for t in self.tracks:
                if not t.alive or id(t) in matched: continue
                tcx, tcy = t.center()
                d = float(np.hypot(du_ - tcx, dv_ - tcy))
                if d < bd: best, bd = t, d
            if best is not None:
                best.refresh(frame, det['bbox'], det['xyz']); matched.add(id(best))
            elif len(self.tracks) < MAX_TRACKS:
                nt = Track(self.next_id, frame, det['bbox']); nt.pose = det['xyz']
                self.tracks.append(nt); matched.add(id(nt))
                self.get_logger().info(f'NEW PROBE id={self.next_id}'); self.next_id += 1
        for t in self.tracks:
            if id(t) in matched: continue
            t.misses += 1
            tcx, tcy = t.center(); Zc = self.depth_point(tcx, tcy)
            if Zc is not None:
                t.pose = ((tcx-CX)*Zc/FX, (tcy-CY)*Zc/FY, Zc)
        self.tracks = [t for t in self.tracks if t.alive and t.misses < MAX_MISSES]

    # ---------- main ----------
    def rgb_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        raw   = cv2.inRange(hsv, LOWER, UPPER)
        mask0 = cv2.morphologyEx(raw,   cv2.MORPH_OPEN,  self.open_k)
        mask  = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, self.close_k)

        self.track(frame, self.detect(frame, mask0, mask))

        poses = PoseArray(); poses.header = msg.header
        lines = []
        for t in self.tracks:
            if t.pose is None: continue
            x, y, z = t.pose
            tx, ty, tw, th = t.bbox
            coasting = t.misses > 0
            col = (0,200,255) if coasting else (0,255,0)
            cv2.rectangle(frame, (tx,ty), (tx+tw,ty+th), col, 2)
            cv2.putText(frame, f'ID {t.id}'+(' (coast)' if coasting else ''),
                        (tx, ty-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.putText(frame, f'X{x:+.2f} Y{y:+.2f} Z{z:.2f}',
                        (tx, ty+th+18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            p = Pose(); p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
            poses.poses.append(p)
            lines.append(f'ID{t.id}: X={x:+.3f} Y={y:+.3f} Z={z:.3f} m'
                         + (' [coast]' if coasting else ''))

        self.pose_pub.publish(poses)
        if lines:
            self.get_logger().info('PROBES  ' + ' | '.join(lines),
                                   throttle_duration_sec=0.5)
        self.publish_img(self.img_pub, frame, msg.header)
        if DEBUG:
            self.publish_img(self.mask_pub, mask, msg.header)

    def publish_img(self, pub, img, header):
        out = CompressedImage(); out.header = header; out.format = 'jpeg'
        out.data = cv2.imencode('.jpg', img)[1].tobytes()
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProbeNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()