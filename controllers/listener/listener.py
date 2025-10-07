from controller import Robot
import json, math

def clamp(v, vmin, vmax): return min(max(v, vmin), vmax)
def wrap_pi(a):  # [-pi, pi)
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

class Follower(Robot):
    # Tunables
    D_DESIRED = 5.0          # keep this many meters away (horizontal)
    DEAD_BAND = 0.20         # don't react to tiny distance errors (m)
    TARGET_ALT = 15.0        # hold this altitude (set to None to follow leader's z)
    PACKET_TIMEOUT = 1.0     # seconds; if stale, hover

    # Gains (conservative; bump slowly if you need more authority)
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    K_YAW = 0.5              # yaw proportional gain (rad -> unit disturbance)
    K_RANGE = 0.12           # pitch disturbance per meter of range error
    MAX_PITCH_DIST = 0.35    # clamp forward/back tilt (|pitch_disturbance|)
    MAX_YAW_DIST = 0.8       # clamp yaw disturbance

    # Optional: only follow a specific leader id (string). Use None to follow the latest packet.
    FOLLOW_ID = None  # e.g., "drone_0" or None

    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        # --- radio ---
        self.rx = self.getDevice("receiver")
        if self.rx is None:
            print("[listener] ERROR: Receiver 'receiver' missing.")
            while self.step(self.dt) != -1:
                pass
            return
        self.rx.enable(self.dt)
        try: self.rx.setChannel(1)
        except Exception: pass

        # --- sensors ---
        self.imu = self.getDevice("inertial unit"); self.imu.enable(self.dt)
        self.gps = self.getDevice("gps"); self.gps.enable(self.dt)
        self.gyro = self.getDevice("gyro"); self.gyro.enable(self.dt)

        # --- motors ---
        self.fl  = self.getDevice("front left propeller")
        self.fr  = self.getDevice("front right propeller")
        self.rl  = self.getDevice("rear left propeller")
        self.rr  = self.getDevice("rear right propeller")
        for m in (self.fl, self.fr, self.rl, self.rr):
            m.setPosition(float('inf')); m.setVelocity(1.0)

        self.last_leader = None   # (x,y,z,yaw,t_sim, id)
        self.alt_target = self.TARGET_ALT

        print("[listener] Follower ready. Keeping distance to broadcaster...")
        self._print_tick = 0
        self._print_every = max(1, int(0.2 * 1000.0 / self.dt))  # ~5 Hz


    # --- radio handling ---
    def _drain_packets(self):
        latest = None
        while self.rx.getQueueLength() > 0:
            raw = self.rx.getString()      # <-- this is already a str
            self.rx.nextPacket()
            try:
                msg = json.loads(raw)      # <-- parse directly
            except Exception:
                # Fallback for backends that only provide bytes:
                try:
                    data = self.rx.getData()
                    msg = json.loads(data.decode("utf-8"))
                except Exception:
                    continue

            if not isinstance(msg, dict) or msg.get("type") != "pose":
                continue
            ident = msg.get("id", "")
            if self.FOLLOW_ID is not None and ident != self.FOLLOW_ID:
                continue
            p = msg.get("p", [])
            if not (isinstance(p, list) and len(p) >= 4):
                continue
            t_msg = msg.get("t", self.getTime())
            latest = (float(p[0]), float(p[1]), float(p[2]), float(p[3]), float(t_msg), str(ident))
        if latest is not None:
            self.last_leader = latest


    def run(self):
        while self.step(self.dt) != -1:
            self._drain_packets()

            # --- read own pose ---
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, z = self.gps.getValues()
            roll_rate, pitch_rate, _ = self.gyro.getValues()

            # --- choose control targets ---
            now = self.getTime()
            yaw_dist = 0.0
            pitch_dist = 0.0

            # altitude: hold constant or match leader if configured
            if self.alt_target is None and self.last_leader:
                self.alt_target = self.last_leader[2]  # follow leader's z
            if self.alt_target is None:
                self.alt_target = z  # freeze at current if still None

            # If we have a fresh leader packet, compute yaw/pitch commands
            # ...
            fresh = False
            if self.last_leader is not None:
                lx, ly, lz, lyaw, t_msg, lid = self.last_leader
                fresh = (now - t_msg) <= self.PACKET_TIMEOUT

                dx, dy = (lx - x), (ly - y)
                dist = math.hypot(dx, dy)          # <-- compute regardless of freshness

                if fresh:
                    bearing = math.atan2(dy, dx)
                    yaw_err = wrap_pi(bearing - yaw)
                    yaw_dist = clamp(self.K_YAW * yaw_err, -self.MAX_YAW_DIST, self.MAX_YAW_DIST)

                    rng_err = dist - self.D_DESIRED
                    if abs(rng_err) > self.DEAD_BAND:
                        pitch_dist = clamp(-self.K_RANGE * rng_err, -self.MAX_PITCH_DIST, self.MAX_PITCH_DIST)
                    else:
                        pitch_dist = 0.0
                else:
                    yaw_dist = 0.0
                    pitch_dist = 0.0

                # Throttled status print (inside the block so vars exist)
                self._print_tick += 1
                if self._print_tick >= self._print_every:
                    self._print_tick = 0
                    status = "fresh" if fresh else "stale"
                    print(f"[listener t={now:.2f}] follow={lid or 'unknown'} | {status} | dist={dist:.2f} m")
            else:
                # Optional: occasional notice when no packets ever seen
                self._print_tick += 1
                if self._print_tick >= 10 * self._print_every:
                    self._print_tick = 0
                    print(f"[listener t={now:.2f}] no leader packets yet...")


            # --- core motor mixing (same style as your other controller) ---
            clamped_roll  = clamp(roll,  -1, 1)
            clamped_pitch = clamp(pitch, -1, 1)
            roll_input  = self.K_ROLL_P  * clamped_roll  + roll_rate
            pitch_input = self.K_PITCH_P * clamped_pitch + pitch_rate + pitch_dist
            yaw_input   = yaw_dist

            # altitude hold
            clamped_diff_alt = clamp((self.alt_target - z) + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * (clamped_diff_alt ** 3.0)

            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.fl.setVelocity(fl)
            self.fr.setVelocity(-fr)
            self.rl.setVelocity(-rl)
            self.rr.setVelocity(rr)

if __name__ == "__main__":
    Follower().run()
