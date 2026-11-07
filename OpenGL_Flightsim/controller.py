#!/usr/bin/env python3
"""
Fixed-wing camera + EKF controller for OpenGL_Flightsim (hackathon UAV mode).

It flies the aircraft into the magenta target sphere CONTINUOUSLY: every time the
simulator scores a hit it respawns the target, and this controller simply keeps
going after the next one (stop with Ctrl+C, --hits N or --max-time S).

How it uses the EKF
-------------------
The camera is fixed to the airframe, so when the aircraft banks the image banks
with it and the pixel position of the target no longer says "turn left/right".
Each detection is turned into a ray in the camera frame, rotated into the body
frame (camera tilt), then rotated into the navigation frame with the EKF attitude
quaternion. From that ray we get
    bearing   : angle of the target right/left of the current heading
    elevation : angle of the target above/below the horizon
and those drive the guidance:
    heading-rate demand = K_PSI * bearing   ->  bank = atan(psi_dot * V / g)
    flight-path demand  = K_GAMMA * elevation ->  pitch command
So a bad attitude estimate shows up directly as a bad trajectory: this is an
end-to-end test of the STM32 EKF. Use --attitude truth to fly the same mission
with the true attitude and compare.

When the target is out of view the controller keeps turning towards where it
last saw it (or sweeps), so a missed pass turns into a loop and another try.

Shared memory: SharedBlock from src/hackathon/sim_types.h (BRIDGE_VERSION == 3).
Works on Windows (named file mapping) and Linux (/dev/shm).
Requires: pip install numpy opencv-python
"""
import argparse
import csv
import math
import mmap
import os
import struct
import sys
import time

import numpy as np
import cv2

# ----------------------------------------------------------------------------
# Must match sim_types.h
# ----------------------------------------------------------------------------
CAM_WIDTH, CAM_HEIGHT = 640, 480
BRIDGE_NAME = "flightsim_bridge"
BRIDGE_VERSION = 3
BRIDGE_MAGIC = 0x55415653

BRIDGE_HEADER_SIZE = 32
SIM_STATE_SIZE = 72
IMU_SAMPLE_SIZE = 48
EKF_STATE_SIZE = 48
CONTROL_CMD_SIZE = 32
TARGET_INFO_SIZE = 28
CAMERA_INFO_SIZE = 40
PIXEL_BYTES = CAM_WIDTH * CAM_HEIGHT * 3

OFF_HEADER = 0
OFF_TRUTH = BRIDGE_HEADER_SIZE
OFF_IMU = OFF_TRUTH + SIM_STATE_SIZE
OFF_EKF = OFF_IMU + IMU_SAMPLE_SIZE
OFF_CMD = OFF_EKF + EKF_STATE_SIZE
OFF_TARGET = OFF_CMD + CONTROL_CMD_SIZE
# CameraInfo starts with a uint64_t, so the C++ compiler aligns it to 8 bytes:
# TargetInfo ends at 260 and CameraInfo starts at 264 (4 bytes of padding).
# The old controller ignored this and read the camera info and every pixel 4 bytes off.
OFF_CAMERA = (OFF_TARGET + TARGET_INFO_SIZE + 7) & ~7      # 264
OFF_PIXELS = OFF_CAMERA + CAMERA_INFO_SIZE                  # 304
SHM_SIZE = OFF_PIXELS + PIXEL_BYTES                         # == sizeof(SharedBlock)

G = 9.81


# ----------------------------------------------------------------------------
# Shared memory
# ----------------------------------------------------------------------------
class Bridge:
    def __init__(self):
        self.mm = None
        self.fd = None
        self.seq = 0

    def open(self, wait_s=60.0):
        """Open the block the simulator created; wait for it if the sim is still starting."""
        t0 = time.time()
        announced = False
        while True:
            try:
                if os.name == "nt":
                    # Opens the existing mapping created by CreateFileMappingA in sim_bridge.cpp
                    self.mm = mmap.mmap(-1, SHM_SIZE, tagname=BRIDGE_NAME)
                else:
                    path = "/dev/shm/" + BRIDGE_NAME
                    self.fd = os.open(path, os.O_RDWR)
                    if os.fstat(self.fd).st_size < SHM_SIZE:
                        raise OSError("block not sized yet")
                    self.mm = mmap.mmap(self.fd, SHM_SIZE)
                magic, version, total, running = struct.unpack_from("<IIII", self.mm, OFF_HEADER)
                if magic == BRIDGE_MAGIC:
                    if version != BRIDGE_VERSION:
                        raise RuntimeError(f"bridge version mismatch: sim={version}, controller={BRIDGE_VERSION}")
                    if total != SHM_SIZE:
                        raise RuntimeError(f"bridge size mismatch: sim={total}, controller={SHM_SIZE}")
                    return
                raise OSError("simulator has not initialised the block yet")
            except (OSError, ValueError) as e:
                self.close()
                if time.time() - t0 > wait_s:
                    raise RuntimeError(f"could not open shared memory '{BRIDGE_NAME}' ({e}). "
                                       "Is the simulator running?")
                if not announced:
                    print("[ctrl] waiting for the simulator ...")
                    announced = True
                time.sleep(0.5)

    def close(self):
        if self.mm is not None:
            try:
                self.mm.close()
            except Exception:
                pass
            self.mm = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    # ---- readers ----
    def header(self):
        magic, version, total, running, t_us, steps = struct.unpack_from("<IIIIQQ", self.mm, OFF_HEADER)
        return dict(running=running, t=t_us * 1e-6, steps=steps)

    def truth(self):
        """Ground truth: logging / scoring only, never used for guidance unless --attitude truth."""
        v = struct.unpack_from("<18f", self.mm, OFF_TRUTH)
        return dict(q=v[0:4], roll=v[4], pitch=v[5], yaw=v[6], w=v[7:10], pos=v[10:13],
                    vel=v[13:16], airspeed=v[16], alt=v[17])

    def ekf(self):
        v = struct.unpack_from("<11fI", self.mm, OFF_EKF)
        return dict(q=v[0:4], roll=v[4], pitch=v[5], yaw=v[6], bias=v[7:10],
                    err=v[10], accel_used=v[11])

    def target(self):
        px, py, pz, radius, dist, hits, last = struct.unpack_from("<5fII", self.mm, OFF_TARGET)
        return dict(pos=(px, py, pz), radius=radius, distance=dist, hits=hits)

    def camera(self):
        frame_id, w, h, c, vfov, tilt, bottom_up, rgb, _ = struct.unpack_from("<QIIIffIII", self.mm, OFF_CAMERA)
        return dict(frame_id=frame_id, vfov=vfov or 60.0, tilt=tilt, bottom_up=bottom_up, rgb=rgb)

    def frame_bgr(self, cam):
        buf = np.frombuffer(self.mm, dtype=np.uint8, count=PIXEL_BYTES, offset=OFF_PIXELS)
        img = buf.reshape((CAM_HEIGHT, CAM_WIDTH, 3))
        if cam["bottom_up"]:
            img = img[::-1]          # OpenGL rows are bottom-up
        if cam["rgb"]:
            img = img[:, :, ::-1]    # RGB -> BGR for OpenCV
        return np.ascontiguousarray(img)

    # ---- writer ----
    def command(self, roll, pitch, speed, armed=1, reset=0):
        self.seq += 1
        struct.pack_into("<QffffII", self.mm, OFF_CMD, self.seq, float(roll), float(pitch),
                         float(speed), -1.0, 1 if armed else 0, reset)


# ----------------------------------------------------------------------------
# Vision
# ----------------------------------------------------------------------------
_KERNEL = np.ones((3, 3), np.uint8)


def detect_target(frame_bgr):
    """Magenta sphere (RGB 255,0,255). Returns (cx, cy, area_px) or None.

    The target is only ~8 px wide at 250 m, so only a light opening is applied
    (the old erode x2 deleted it completely at long range)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([140, 120, 80], np.uint8), np.array([165, 255, 255], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        # opening can still kill a 2-3 px speck at extreme range; fall back to the raw mask
        mask = cv2.inRange(hsv, np.array([140, 120, 80], np.uint8), np.array([165, 255, 255], np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        if n <= 1:
            return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < 3:
        return None
    return float(cents[i][0]), float(cents[i][1]), area


# ----------------------------------------------------------------------------
# Geometry: pixel -> ray -> navigation frame via the attitude quaternion
# ----------------------------------------------------------------------------
def quat_rotate(q, v):
    """Rotate v by unit quaternion q = (w, x, y, z) (body -> nav)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def pixel_to_body_ray(cx, cy, vfov_deg, tilt_deg):
    """Pixel (top-down image) -> unit ray in the FLU body frame (x fwd, y left, z up)."""
    ty = math.tan(math.radians(vfov_deg) / 2.0)
    tx = ty * CAM_WIDTH / CAM_HEIGHT
    u = (cx - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0) * tx     # + right
    v = -(cy - CAM_HEIGHT / 2.0) / (CAM_HEIGHT / 2.0) * ty  # + up
    t = math.radians(tilt_deg)
    fwd = (math.cos(t), 0.0, math.sin(t))
    up = (-math.sin(t), 0.0, math.cos(t))
    right = (0.0, -1.0, 0.0)
    r = tuple(fwd[i] + u * right[i] + v * up[i] for i in range(3))
    n = math.sqrt(sum(c * c for c in r))
    return tuple(c / n for c in r)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def los_angles(q_flu, ray_body, yaw_aircraft):
    """Bearing (+ right of heading) and elevation (+ above horizon) of the line of sight."""
    rn = quat_rotate(q_flu, ray_body)
    az_flu = math.atan2(rn[1], rn[0])           # + to the left (FLU)
    bearing = wrap(-az_flu - yaw_aircraft)      # aircraft yaw: + right
    elevation = math.atan2(rn[2], math.hypot(rn[0], rn[1]))
    return bearing, elevation


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--show", action="store_true", help="camera window with detection overlay")
    ap.add_argument("--hits", type=int, default=0, help="stop after N hits (0 = run forever)")
    ap.add_argument("--max-time", type=float, default=0.0, help="stop after S seconds of sim time (0 = forever)")
    ap.add_argument("--reset", action="store_true", help="ask the sim to reset UAV + target at start")
    ap.add_argument("--attitude", choices=["ekf", "truth"], default="ekf",
                    help="attitude used for guidance (truth = A/B baseline for the EKF)")
    ap.add_argument("--speed", type=float, default=18.0, help="cruise airspeed setpoint, m/s")
    ap.add_argument("--log", default="", help="CSV log of truth vs EKF attitude, guidance and range")
    args = ap.parse_args()

    # guidance gains / limits
    K_PSI = 1.6                      # heading-rate demand per rad of bearing, 1/s
    K_PSI_D = 0.4                    # damping on bearing rate
    K_GAMMA = 1.8                    # flight-path demand per rad of elevation
    MAX_BANK = math.radians(45.0)
    MAX_PITCH = math.radians(25.0)
    SEARCH_BANK = math.radians(30.0)
    LOST_HOLD_S = 2.5                # keep steering on the last LOS this long after losing it

    br = Bridge()
    br.open()
    print(f"[ctrl] bridge OK (v{BRIDGE_VERSION}, {SHM_SIZE} bytes), guidance attitude = {args.attitude.upper()}")
    print("[ctrl] flying continuously towards the target. Ctrl+C to stop.")

    if args.reset:
        br.command(0, 0, args.speed, armed=1, reset=1)
        time.sleep(0.2)

    logf = None
    logw = None
    if args.log:
        logf = open(args.log, "w", newline="")
        logw = csv.writer(logf)
        logw.writerow(["t", "hits", "range_m", "seen", "bearing_deg", "elev_deg", "roll_cmd_deg",
                       "pitch_cmd_deg", "true_roll", "true_pitch", "true_yaw", "ekf_roll", "ekf_pitch",
                       "ekf_yaw", "ekf_err_deg", "accel_used"])

    start_hits = br.target()["hits"]
    last_hits = start_hits
    t_start = br.header()["t"]
    t_last_hit = t_start
    last_seen_t = -1e9
    last_bearing = 0.0
    last_elev = 0.0
    prev_bearing = None
    prev_t = None
    bearing_rate = 0.0
    err_sum = 0.0
    err_n = 0
    err_max = 0.0
    pass_err_sum = 0.0
    pass_err_n = 0
    last_print = 0.0
    last_sim_t = -1.0
    stall_since = time.time()

    try:
        while True:
            hdr = br.header()
            t = hdr["t"]
            if t == last_sim_t:
                if time.time() - stall_since > 5.0:
                    if not hdr["running"]:
                        print("[ctrl] simulator exited.")
                        break
                    # paused (P key) or still loading; keep the link alive
                    stall_since = time.time()
            else:
                stall_since = time.time()
            last_sim_t = t

            cam = br.camera()
            ekf = br.ekf()
            tru = br.truth()
            tgt = br.target()

            att = ekf if args.attitude == "ekf" else tru
            q_att, yaw_att = att["q"], att["yaw"]

            # EKF statistics (diagnostic, uses the sim's truth comparison)
            err_sum += ekf["err"]
            err_n += 1
            pass_err_sum += ekf["err"]
            pass_err_n += 1
            err_max = max(err_max, ekf["err"])

            frame = br.frame_bgr(cam)
            det = detect_target(frame)
            V = max(tru["airspeed"], 8.0)   # pitot airspeed (a real sensor on the aircraft)

            mode = "SEARCH"
            if det is not None:
                cx, cy, area = det
                ray = pixel_to_body_ray(cx, cy, cam["vfov"], cam["tilt"])
                bearing, elev = los_angles(q_att, ray, yaw_att)
                if prev_bearing is not None and t > prev_t:
                    raw = wrap(bearing - prev_bearing) / (t - prev_t)
                    bearing_rate += (clamp(raw, -3, 3) - bearing_rate) * 0.3
                prev_bearing, prev_t = bearing, t
                last_seen_t, last_bearing, last_elev = t, bearing, elev
                mode = "TRACK"
            else:
                prev_bearing = None
                bearing_rate *= 0.9
                if t - last_seen_t < LOST_HOLD_S:
                    # extrapolate: we turned since we last saw it
                    bearing, elev = last_bearing, last_elev
                    mode = "COAST"

            if mode in ("TRACK", "COAST"):
                psi_dot = K_PSI * bearing + K_PSI_D * bearing_rate
                roll_cmd = clamp(math.atan(psi_dot * V / G), -MAX_BANK, MAX_BANK)
                pitch_cmd = clamp(K_GAMMA * elev + 0.02, -MAX_PITCH, MAX_PITCH)
            else:
                # Lost it: keep turning towards the side it was last seen on (a missed pass
                # becomes a loop back), hold altitude roughly level.
                side = 1.0 if last_bearing >= 0 else -1.0
                roll_cmd = side * SEARCH_BANK
                pitch_cmd = clamp(0.8 * last_elev, -math.radians(8), math.radians(8)) if t - last_seen_t < 15 else 0.03

            br.command(roll_cmd, pitch_cmd, args.speed, armed=1)

            # ---- hit bookkeeping (the sim scores and respawns the target) ----
            if tgt["hits"] < last_hits:           # sim was reset (R key / --reset)
                start_hits = last_hits = tgt["hits"]
                t_last_hit = t
            if tgt["hits"] > last_hits:
                n = tgt["hits"] - start_hits
                dt_hit = t - t_last_hit
                print(f"[ctrl] HIT #{n:3d}  t={t - t_start:7.1f}s  (+{dt_hit:5.1f}s)  "
                      f"EKF err this pass mean {pass_err_sum / max(pass_err_n, 1):5.2f} deg, "
                      f"overall mean {err_sum / max(err_n, 1):5.2f} / max {err_max:5.2f} deg")
                last_hits = tgt["hits"]
                t_last_hit = t
                pass_err_sum, pass_err_n = 0.0, 0
                last_seen_t = -1e9
                if args.hits and n >= args.hits:
                    print(f"[ctrl] reached {n} hits, done.")
                    break

            if args.max_time and t - t_start >= args.max_time:
                print("[ctrl] max time reached.")
                break

            if logw is not None:
                d = math.degrees
                logw.writerow([f"{t:.3f}", tgt["hits"] - start_hits, f"{tgt['distance']:.2f}",
                               int(det is not None),
                               f"{d(bearing):.2f}" if mode != "SEARCH" else "",
                               f"{d(elev):.2f}" if mode != "SEARCH" else "",
                               f"{d(roll_cmd):.2f}", f"{d(pitch_cmd):.2f}",
                               f"{d(tru['roll']):.2f}", f"{d(tru['pitch']):.2f}", f"{d(tru['yaw']):.2f}",
                               f"{d(ekf['roll']):.2f}", f"{d(ekf['pitch']):.2f}", f"{d(ekf['yaw']):.2f}",
                               f"{ekf['err']:.3f}", ekf["accel_used"]])

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"[ctrl] t={t - t_start:6.1f}s {mode:6s} range={tgt['distance']:6.1f}m "
                      f"bank_cmd={math.degrees(roll_cmd):5.1f} pitch_cmd={math.degrees(pitch_cmd):5.1f} "
                      f"EKF err={ekf['err']:5.2f}deg hits={tgt['hits'] - start_hits}")

            if args.show:
                vis = frame.copy()
                cv2.drawMarker(vis, (CAM_WIDTH // 2, CAM_HEIGHT // 2), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
                if det is not None:
                    r = max(6, int(math.sqrt(det[2] / math.pi)) + 4)
                    cv2.circle(vis, (int(det[0]), int(det[1])), r, (0, 255, 255), 2)
                cv2.putText(vis, f"{mode} hits {tgt['hits'] - start_hits}  EKF err {ekf['err']:.1f} deg",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                cv2.imshow("onboard camera", vis)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

            time.sleep(0.01)  # ~100 Hz

    except KeyboardInterrupt:
        print("\n[ctrl] stopped.")
    finally:
        n = br.target()["hits"] - start_hits if br.mm else 0
        print(f"[ctrl] summary: {n} hits, EKF attitude error mean {err_sum / max(err_n, 1):.2f} deg, "
              f"max {err_max:.2f} deg")
        if br.mm is not None:
            # hand control back to the sim's failsafe (level flight)
            br.command(0, 0, args.speed, armed=0)
        if logf:
            logf.close()
        br.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
