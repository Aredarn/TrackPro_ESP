#!/usr/bin/env python3
"""
TrackPro GPS simulator.

Stands in for the ESP32 firmware (TrackPro_ESP32_TPC_NO_OLED.ino) over WiFi,
so the Android app's lap timer and drag ("dragy") features can be exercised
without real hardware or driving. Speaks the exact same line-delimited JSON
+ RATE command protocol as the firmware: connect the app to this machine's
IP on port 4210 the same way it would connect to the ESP32's AP.

Two modes:
  lap   Loops a closed track polygon repeatedly. Speed is derived from the
        turn angle at each waypoint (slower in corners, faster on straights)
        and position advances by actual distance-at-speed each tick, so lap
        time and the reported speed trace are self-consistent - not just a
        dot moving on a map, but something a lap timer can meaningfully time.
  drag  A straight-line run from a standstill: idles at speed 0, launches
        with a configurable accel curve toward a target top speed, covers a
        configurable distance (default 402m / 1/4 mile), then coasts down
        and loops back to the line. For testing 0-60 / quarter-mile timing
        against a known, reproducible reference.

Examples:
  python esp32_simulator.py lap --track nurburgring --hz 20
  python esp32_simulator.py drag --heading 90 --top-speed 220 --accel-time 4.5
  python esp32_simulator.py lap --dropout-rate 0.03   # inject occasional fix loss

Bluetooth is intentionally not simulated here: a Windows PC acting as the
Bluetooth Classic (SPP) *server* the way the ESP32 does is fragile (PyBluez
is unmaintained and often won't build on recent Python/Windows). Test the
Bluetooth path with the real ESP32 board, which already speaks real SPP.
"""

import argparse
import json
import math
import random
import select
import socket
import sys
import time

sys.stdout.reconfigure(line_buffering=True)  # so logs show up live when piped to a file, not just a TTY

HOST = "0.0.0.0"
PORT = 4210
SUPPORTED_RATES_HZ = (5, 10, 20, 25)
EARTH_RADIUS_M = 6371000.0

# ---------------------------------------------------------------------------
# Track waypoint data (closed loops, lat/lon)
# ---------------------------------------------------------------------------

TRACKS = {
    "pannonia": [
        (47.305300, 17.048138), (47.302270, 17.049691), (47.301004, 17.048439),
        (47.300890, 17.048053), (47.301029, 17.047764), (47.302883, 17.046380),
        (47.302997, 17.046187), (47.303095, 17.045789), (47.303422, 17.043212),
        (47.303258, 17.042779), (47.302997, 17.042706), (47.300882, 17.045801),
        (47.300572, 17.045994), (47.300237, 17.045910), (47.299959, 17.045356),
        (47.299992, 17.044838), (47.301160, 17.040972), (47.301486, 17.040623),
        (47.301846, 17.040563), (47.302744, 17.041346), (47.302981, 17.041382),
        (47.303307, 17.041213), (47.303724, 17.040515), (47.303887, 17.039732),
        (47.303691, 17.037420), (47.303691, 17.037046), (47.304508, 17.035324),
        (47.304696, 17.035192), (47.304851, 17.035276), (47.305349, 17.036324),
        (47.305373, 17.036757), (47.305210, 17.037335), (47.304843, 17.037733),
        (47.304638, 17.038239), (47.304565, 17.038624), (47.304508, 17.043248),
        (47.304451, 17.043513), (47.304075, 17.044344), (47.304026, 17.044549),
        (47.304059, 17.044838), (47.304181, 17.045151), (47.304393, 17.045187),
        (47.304557, 17.045139), (47.305594, 17.044380), (47.306631, 17.042670),
        (47.306778, 17.042550), (47.306958, 17.042550), (47.307137, 17.042622),
        (47.308574, 17.044501), (47.308648, 17.044730), (47.308721, 17.045585),
        (47.308615, 17.046054), (47.308436, 17.046392), (47.306092, 17.047740),
        (47.304973, 17.048282),
    ],
    "nurburgring": [
        (50.3527008, 6.9832441), (50.3500245, 6.9762865), (50.346916, 6.968372),
        (50.345397, 6.964457), (50.3439256, 6.9609668), (50.3431563, 6.9592267),
        (50.3408439, 6.9562155), (50.340609, 6.9559632), (50.3403983, 6.9557741),
        (50.339538, 6.9549679), (50.3388295, 6.9538264), (50.3383564, 6.9534466),
        (50.3383033, 6.9534156), (50.338075, 6.9528681), (50.3377306, 6.9512265),
        (50.3377353, 6.9509513), (50.3382678, 6.9498572), (50.3385235, 6.949453),
        (50.3386769, 6.9492416), (50.3391848, 6.9486675), (50.3393483, 6.948297),
        (50.3393488, 6.9482718), (50.3392871, 6.9480445), (50.3392658, 6.9480015),
        (50.3382658, 6.9470826), (50.337688, 6.9461167), (50.3373742, 6.9450709),
        (50.3373435, 6.94492), (50.3373243, 6.9443909), (50.3373375, 6.944251),
        (50.3379549, 6.9421846), (50.3380658, 6.9413132), (50.3381323, 6.9399485),
        (50.3379337, 6.9390216), (50.3379374, 6.9389073), (50.3379504, 6.9388018),
        (50.3383435, 6.9380255), (50.3384501, 6.9379093), (50.3387776, 6.9376799),
        (50.338898, 6.9375762), (50.3389569, 6.9375013), (50.3395809, 6.9364737),
        (50.3399589, 6.9361394), (50.3402372, 6.9339092), (50.340274, 6.9338072),
        (50.3403501, 6.9336818), (50.3410153, 6.9334324), (50.3413213, 6.9330873),
        (50.3428825, 6.9295538), (50.3433293, 6.9287777), (50.3451554, 6.9264573),
        (50.3454649, 6.9260815), (50.3455173, 6.9260372), (50.3466946, 6.9258378),
        (50.3467818, 6.9258729), (50.3480198, 6.9266375), (50.3488714, 6.9268463),
        (50.3494568, 6.926965), (50.351027, 6.9268148), (50.3511836, 6.9267751),
        (50.3538457, 6.9252582), (50.3545279, 6.9248292), (50.3556528, 6.924118),
        (50.3563301, 6.9233907), (50.356918, 6.9221102), (50.3574215, 6.920398),
        (50.358063, 6.920062), (50.358326, 6.9203826), (50.3587091, 6.984639),
        (50.3588149, 6.9844146), (50.3589474, 6.9838568), (50.3589354, 6.9836354),
        (50.3578657, 6.9811965), (50.3578266, 6.9811722), (50.3577884, 6.9811555),
        (50.357486, 6.9812753), (50.3571103, 6.9818028), (50.356904, 6.982205),
        (50.3561681, 6.9852577), (50.3560985, 6.985395), (50.3555808, 6.9859393),
        (50.3555271, 6.9859807), (50.3547522, 6.9863194), (50.3541198, 6.9862103),
        (50.3537712, 6.9859117), (50.3534631, 6.9854013), (50.3527008, 6.9832441),
    ],
}

# ---------------------------------------------------------------------------
# Geo helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(x, y)) % 360


def destination_point(lat, lon, bearing, distance_m):
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = math.radians(bearing)
    d_r = distance_m / EARTH_RADIUS_M
    lat2 = math.asin(math.sin(lat1) * math.cos(d_r) + math.cos(lat1) * math.sin(d_r) * math.cos(brng))
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(d_r) * math.cos(lat1),
        math.cos(d_r) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def turn_angle_deg(bearing_a, bearing_b):
    return abs((bearing_b - bearing_a + 540) % 360 - 180)

# ---------------------------------------------------------------------------
# Lap mode
# ---------------------------------------------------------------------------

class LapSource:
    def __init__(self, track_name, max_speed_kmh, min_corner_speed_kmh):
        self.points = TRACKS[track_name]
        n = len(self.points)
        bearings = [bearing_deg(*self.points[i], *self.points[(i + 1) % n]) for i in range(n)]
        # Target speed AT waypoint i, based on how sharply the path turns there
        # (angle between the incoming and outgoing segment) - sharper turn,
        # lower target speed, clamped and linearly scaled between the two
        # configured bounds.
        self.target_speed = []
        for i in range(n):
            angle = turn_angle_deg(bearings[(i - 1) % n], bearings[i])
            frac = min(angle, 90.0) / 90.0
            self.target_speed.append(max_speed_kmh - frac * (max_speed_kmh - min_corner_speed_kmh))
        self.seg_len_m = [haversine_m(*self.points[i], *self.points[(i + 1) % n]) for i in range(n)]
        self.reset()

    def reset(self):
        self.idx = 0
        self.dist_into_seg = 0.0
        self.current_speed = self.target_speed[0]
        self.altitude = 150.0

    def step(self, dt):
        # Ease current speed toward the target speed at the end of this
        # segment instead of snapping, so the reported speed trace looks
        # like a car braking/accelerating rather than teleporting.
        target = self.target_speed[(self.idx + 1) % len(self.points)]
        accel = 25.0 if target > self.current_speed else 40.0  # km/h per second; brakes harder than it accelerates
        if self.current_speed < target:
            self.current_speed = min(target, self.current_speed + accel * dt)
        else:
            self.current_speed = max(target, self.current_speed - accel * dt)

        self.dist_into_seg += self.current_speed * 1000.0 / 3600.0 * dt

        seg_len = self.seg_len_m[self.idx]
        while seg_len > 0 and self.dist_into_seg >= seg_len:
            self.dist_into_seg -= seg_len
            self.idx = (self.idx + 1) % len(self.points)
            seg_len = self.seg_len_m[self.idx]

        frac = 0.0 if seg_len == 0 else self.dist_into_seg / seg_len
        lat1, lon1 = self.points[self.idx]
        lat2, lon2 = self.points[(self.idx + 1) % len(self.points)]
        lat = lat1 + (lat2 - lat1) * frac
        lon = lon1 + (lon2 - lon1) * frac
        self.altitude += random.uniform(-0.3, 0.3)
        return lat, lon, self.current_speed, self.altitude

# ---------------------------------------------------------------------------
# Drag mode
# ---------------------------------------------------------------------------

class DragSource:
    def __init__(self, start_lat, start_lon, heading, top_speed_kmh, accel_time_s, distance_m, idle_s):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.heading = heading
        self.top_speed = top_speed_kmh
        self.distance_m = distance_m
        self.idle_s = idle_s
        # v(t) = top_speed * (1 - exp(-t/tau)), solved so v(accel_time_s) hits
        # ~100 km/h (or 90% of top speed, if top speed is lower than that) -
        # a smooth, plausible accel curve rather than a straight ramp.
        target_v = min(100.0, top_speed_kmh * 0.9)
        self.tau = -accel_time_s / math.log(max(1e-3, 1 - target_v / top_speed_kmh))
        self.reset()

    def reset(self):
        self.phase = "idle"  # idle -> launch -> coast -> (loops back to idle)
        self.t = 0.0
        self.traveled_m = 0.0
        self.altitude = 150.0

    def step(self, dt):
        self.t += dt
        if self.phase == "idle":
            speed = 0.0
            if self.t >= self.idle_s:
                self.phase, self.t = "launch", 0.0
        elif self.phase == "launch":
            speed = self.top_speed * (1 - math.exp(-self.t / self.tau))
            self.traveled_m += speed * 1000.0 / 3600.0 * dt
            if self.traveled_m >= self.distance_m:
                self.phase, self.t = "coast", 0.0
                self.peak_speed = speed
        else:  # coast: brake to a stop, hold briefly, then loop back to the line
            speed = max(0.0, self.peak_speed - 80.0 * self.t)  # 80 km/h/s braking
            if self.t >= 4.0:
                self.reset()

        lat, lon = destination_point(self.start_lat, self.start_lon, self.heading, self.traveled_m)
        self.altitude += random.uniform(-0.2, 0.2)
        return lat, lon, speed, self.altitude

# ---------------------------------------------------------------------------
# Wire protocol - matches TrackPro_ESP32_TPC_NO_OLED.ino exactly
# ---------------------------------------------------------------------------

def make_json_line(lat, lon, alt, speed_kmh, satellites, valid, sim_time_s):
    h = int(sim_time_s // 3600) % 24
    m = int((sim_time_s % 3600) // 60)
    s = int(sim_time_s % 60)
    cc = int((sim_time_s * 100) % 100)
    payload = {
        "latitude": round(lat, 6),
        "longitude": round(lon, 6),
        "altitude": round(alt, 1),
        "speed": round(speed_kmh, 2),
        "satellites": satellites,
        "valid": valid,
        "timestamp": f"{h}:{m}:{s}.{cc}",
    }
    return (json.dumps(payload) + "\n").encode("ascii")


def handle_command(line, current_hz):
    """Mirrors handleCommandLine() in the firmware: RATE:<hz> -> RATE_OK:<hz> or RATE_ERR."""
    line = line.strip()
    if line.startswith("RATE:"):
        try:
            hz = int(line[len("RATE:"):])
        except ValueError:
            hz = -1
        if hz in SUPPORTED_RATES_HZ:
            return hz, f"RATE_OK:{hz}\n".encode("ascii")
    return current_hz, b"RATE_ERR\n"

# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------

def run_server(make_source, initial_hz, dropout_rate):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen()
        print(f"TrackPro simulator listening on {HOST}:{PORT} (WiFi/TCP) - Ctrl+C to stop")

        while True:
            conn, addr = srv.accept()
            conn.setblocking(False)
            print(f"Client connected: {addr}")
            source = make_source()
            hz = initial_hz
            recv_buf = b""
            sim_time = 0.0
            last_send = time.monotonic()

            try:
                while True:
                    period = 1.0 / hz
                    remaining = period - (time.monotonic() - last_send)
                    readable, _, _ = select.select([conn], [], [], max(0.0, remaining))

                    if readable:
                        chunk = conn.recv(256)
                        if not chunk:
                            print("Client disconnected")
                            break
                        recv_buf += chunk
                        while b"\n" in recv_buf:
                            raw_line, recv_buf = recv_buf.split(b"\n", 1)
                            hz, reply = handle_command(raw_line.decode("ascii", "ignore"), hz)
                            conn.sendall(reply)
                            print(f"  <- {raw_line!r}  -> {reply!r}")

                    now = time.monotonic()
                    if now - last_send >= period:
                        dt = now - last_send
                        last_send = now
                        sim_time += dt
                        lat, lon, speed_kmh, alt = source.step(dt)
                        valid = not (dropout_rate > 0 and random.random() < dropout_rate)
                        satellites = random.randint(7, 12) if valid else 0
                        conn.sendall(make_json_line(lat, lon, alt, speed_kmh, satellites, valid, sim_time))
            except OSError as e:
                print(f"Connection ended: {e}")
            finally:
                conn.close()

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--hz", type=int, default=10, choices=SUPPORTED_RATES_HZ,
                         help="Initial update rate (the app can still change it live via Settings)")
    common.add_argument("--dropout-rate", type=float, default=0.0,
                         help="Probability per update (0-1) of a simulated GPS fix loss - tests the app's "
                              "handling of \"valid\": false without it snapping to null-island/0 speed")

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    lap_p = sub.add_parser("lap", parents=[common], help="Loop a track polygon repeatedly (lap timer testing)")
    lap_p.add_argument("--track", choices=sorted(TRACKS), default="nurburgring")
    lap_p.add_argument("--max-speed", type=float, default=180.0, help="Straight-line target speed, km/h")
    lap_p.add_argument("--min-corner-speed", type=float, default=45.0, help="Tightest-corner target speed, km/h")

    drag_p = sub.add_parser("drag", parents=[common], help="Straight-line acceleration run (dragy testing)")
    drag_p.add_argument("--start-lat", type=float, default=47.305300)
    drag_p.add_argument("--start-lon", type=float, default=17.048138)
    drag_p.add_argument("--heading", type=float, default=90.0, help="Compass bearing of the run, degrees")
    drag_p.add_argument("--top-speed", type=float, default=250.0, help="km/h")
    drag_p.add_argument("--accel-time", type=float, default=4.0,
                         help="Seconds to reach ~100 km/h (or 90%% of top speed, if lower)")
    drag_p.add_argument("--distance", type=float, default=402.0, help="Run length, meters (402 = 1/4 mile)")
    drag_p.add_argument("--idle", type=float, default=4.0, help="Seconds stopped at the line before each launch")

    return parser


def main():
    args = build_parser().parse_args()

    if args.mode == "lap":
        make_source = lambda: LapSource(args.track, args.max_speed, args.min_corner_speed)
        print(f"Mode: lap  track={args.track}  max_speed={args.max_speed}km/h  "
              f"min_corner_speed={args.min_corner_speed}km/h")
    else:
        make_source = lambda: DragSource(
            args.start_lat, args.start_lon, args.heading,
            args.top_speed, args.accel_time, args.distance, args.idle,
        )
        print(f"Mode: drag  heading={args.heading}deg  top_speed={args.top_speed}km/h  "
              f"accel_time={args.accel_time}s  distance={args.distance}m  idle={args.idle}s")

    try:
        run_server(make_source, args.hz, args.dropout_rate)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
