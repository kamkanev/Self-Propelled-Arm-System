import time


class BaseController(object):
    def __init__(self, config):
        self.config = config
        self.robot = None

    def connect(self):
        if self.config.get("runtime.dry_run.base", True):
            print("[base] dry_run=True")
            return None
        if self.robot is None:
            from jetbot import Robot

            self.robot = Robot()
            print("[base] Robot connected")
        return self.robot

    def stop(self):
        if self.robot is not None:
            self.robot.stop()
        print("[base] stop")

    def pulse(self, direction, speed, seconds, label):
        bot = self.connect()
        print("[base] {} direction={} speed={} seconds={}".format(label, direction, speed, seconds))
        if bot is None:
            return

        try:
            if direction == "forward":
                bot.forward(float(speed))
            elif direction == "backward":
                bot.backward(float(speed))
            elif direction == "left":
                bot.left(float(speed))
            elif direction == "right":
                bot.right(float(speed))
            else:
                raise ValueError("unknown base direction: {}".format(direction))
            time.sleep(float(seconds))
        finally:
            bot.stop()

    def drive_until(self, direction, speed, stop_check, timeout_seconds, poll_seconds, label):
        bot = self.connect()
        print(
            "[base] {} direction={} speed={} timeout_seconds={} poll_seconds={}".format(
                label,
                direction,
                speed,
                timeout_seconds,
                poll_seconds,
            )
        )

        def drive(value):
            if direction == "forward":
                bot.forward(float(value))
            elif direction == "backward":
                bot.backward(float(value))
            elif direction == "left":
                bot.left(float(value))
            elif direction == "right":
                bot.right(float(value))
            else:
                raise ValueError("unknown base direction: {}".format(direction))

        start = time.time()
        timeout_seconds = max(0.1, float(timeout_seconds))
        poll_seconds = max(0.02, float(poll_seconds))
        try:
            if bot is not None:
                drive(float(speed))
            while time.time() - start < timeout_seconds:
                if stop_check():
                    print("[base] {} stop condition reached elapsed={:.2f}s".format(label, time.time() - start))
                    return True
                time.sleep(poll_seconds)
            print("[base] {} timeout reached elapsed={:.2f}s".format(label, time.time() - start))
            return False
        finally:
            if bot is not None:
                bot.stop()

    def smooth_pulse(self, direction, speed, seconds, label, ramp_steps=4):
        bot = self.connect()
        ramp_steps = max(1, int(ramp_steps))
        print(
            "[base] {} direction={} speed={} seconds={} ramp_steps={}".format(
                label,
                direction,
                speed,
                seconds,
                ramp_steps,
            )
        )
        if bot is None:
            return

        def drive(value):
            if direction == "forward":
                bot.forward(float(value))
            elif direction == "backward":
                bot.backward(float(value))
            elif direction == "left":
                bot.left(float(value))
            elif direction == "right":
                bot.right(float(value))
            else:
                raise ValueError("unknown base direction: {}".format(direction))

        try:
            total = max(0.0, float(seconds))
            speed = float(speed)
            ramp_time = min(total * 0.35, total / 2.0)
            hold_time = max(0.0, total - 2.0 * ramp_time)
            step_time = ramp_time / float(ramp_steps) if ramp_steps else 0.0

            for step in range(1, ramp_steps + 1):
                drive(speed * float(step) / float(ramp_steps))
                time.sleep(step_time)
            if hold_time > 0:
                drive(speed)
                time.sleep(hold_time)
            for step in range(ramp_steps - 1, 0, -1):
                drive(speed * float(step) / float(ramp_steps))
                time.sleep(step_time)
        finally:
            bot.stop()


class ArmController(object):
    def __init__(self, config):
        self.config = config
        self.settings = config.section("arm")
        self.ttl = None

    def connect(self):
        if self.config.get("runtime.dry_run.arm", True):
            print("[arm] dry_run=True")
            return None
        if self.ttl is None:
            from SCSCtrl import TTLServo

            self.ttl = TTLServo
            print("[arm] TTLServo connected")
        return self.ttl

    def move_servo(self, servo_id, angle, speed, label):
        ttl = self.connect()
        print("[arm] {} servo={} angle={} speed={}".format(label, servo_id, angle, speed))
        target_position = None
        if ttl is not None:
            target_position = ttl.servoAngleCtrl(int(servo_id), int(angle), 1, int(speed))
            time.sleep(float(self.settings.get("servo_settle_seconds", 0.35)))
        return target_position

    def pose(self, name, pose=None):
        definition = self.settings["poses"][name]
        pose = pose or definition["angles"]
        speed = int(self.settings["speed"])
        order = definition.get("order", [5, 4, 3, 2, 1])
        target_positions = {}
        print("[arm] pose {}".format(name))
        for sid in order:
            key = "s{}".format(sid)
            if key in pose:
                target = self.move_servo(sid, pose[key], speed, name)
                if target is not None:
                    target_positions[int(sid)] = int(target)
        if self.ttl is not None:
            time.sleep(float(definition.get("pause_seconds", 0.0)))
        return target_positions

    def wait_for_positions(self, target_positions, label):
        if self.config.get("runtime.dry_run.arm", True):
            print("[arm-position] {} dry_run=True; position wait skipped".format(label))
            return True
        wait = self.settings.get("position_wait", {})
        if not bool(wait.get("enabled", True)):
            print("[arm-position] {} disabled; position wait skipped".format(label))
            return True
        if not target_positions:
            print("[arm-position] {} has no raw targets".format(label))
            return False

        ttl = self.connect()
        stability_delta = int(wait.get("stability_delta_raw", 3))
        timeout_seconds = float(wait.get("timeout_seconds", 10.0))
        min_wait_seconds = float(wait.get("min_wait_seconds", 0.5))
        poll_seconds = float(wait.get("poll_seconds", 0.15))
        stable_required = max(1, int(wait.get("stable_samples", 2)))
        stable_samples = 0
        start = time.time()
        last_positions = {}
        previous_positions = None

        print(
            "[arm-position] {} waiting for motion to settle servos={} stability_delta={} min_wait={} timeout={} stable_samples={}".format(
                label,
                sorted(target_positions),
                stability_delta,
                min_wait_seconds,
                timeout_seconds,
                stable_required,
            )
        )
        while time.time() - start < timeout_seconds:
            read_errors = []
            last_positions = {}
            for servo_id in sorted(target_positions):
                try:
                    current = int(ttl.nowPosUpdate(int(servo_id)))
                    last_positions[int(servo_id)] = current
                except Exception as exc:
                    read_errors.append(int(servo_id))
                    last_positions[int(servo_id)] = "error:{}".format(exc)

            movement = {}
            motion_stable = previous_positions is not None and not read_errors
            if previous_positions is not None:
                for servo_id, current in last_positions.items():
                    previous = previous_positions.get(servo_id)
                    if not isinstance(current, int) or not isinstance(previous, int):
                        motion_stable = False
                        continue
                    delta = abs(current - previous)
                    movement[servo_id] = delta
                    if delta > stability_delta:
                        motion_stable = False

            elapsed = time.time() - start
            if motion_stable and elapsed >= min_wait_seconds:
                stable_samples += 1
                if stable_samples >= stable_required:
                    print(
                        "[arm-position] {} settled positions={} movement={} elapsed={:.2f}s".format(
                            label,
                            last_positions,
                            movement,
                            time.time() - start,
                        )
                    )
                    return True
            else:
                stable_samples = 0
                print(
                    "[arm-position] {} waiting positions={} movement={} read_errors={}".format(
                        label,
                        last_positions,
                        movement,
                        read_errors,
                    )
                )
            previous_positions = dict(last_positions)
            time.sleep(max(0.02, poll_seconds))

        print(
            "[arm-position] {} motion did not settle before timeout positions={}; base push blocked".format(
                label,
                last_positions,
            )
        )
        return False
