#!/usr/bin/env python3
"""
go2_webrtc_bridge — cmd_vel_bridge_node
========================================
ROS 2 node that subscribes to /cmd_vel (geometry_msgs/Twist) and forwards
velocity commands to the Unitree GO2 robot via WebRTC, using the same
connection logic as joystick.py.

Additional ROS 2 services
--------------------------
/set_led   (go2_interfaces/srv/SetLed)   — Change VUI LED colour (index 0-6)
/set_gait  (go2_interfaces/srv/SetGait)  — Switch locomotion gait (0-4)
/set_pose  (go2_interfaces/srv/SetPose)  — StandUp (true) / StandDown (false)

Each service returns:
    bool   success   — True on successful WebRTC dispatch
    string message   — Human-readable status / error description

Parameters
----------
robot_ip        : str   — Robot IP address (default: 192.168.123.161)
cmd_timeout     : float — Seconds before a single command times out (default: 5.0)
reconnect_delay : float — Seconds between reconnection attempts (default: 3.0)
max_speed       : float — Maximum speed scaling factor (default: 1.0)
"""

import asyncio
import concurrent.futures
import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------------------------------------------------------------------------
# Attempt to import the go2_webrtc_driver.
# ---------------------------------------------------------------------------
try:
    from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
    from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, VUI_COLOR
    GO2_DRIVER_AVAILABLE = True
except ImportError as _e:
    GO2_DRIVER_AVAILABLE = False
    _IMPORT_ERROR = str(_e)

# ---------------------------------------------------------------------------
# Custom service interfaces
# ---------------------------------------------------------------------------
from go2_interfaces.srv import SetLed, SetGait, SetPose  # noqa: E402


# ---------------------------------------------------------------------------
# ConnectionManager — adapted from joystick.py
# ---------------------------------------------------------------------------
class ConnectionManager:
    """Manages the WebRTC connection with automatic reconnection."""

    def __init__(self, ip: str, cmd_timeout: float, reconnect_delay: float, logger):
        self.ip = ip
        self.cmd_timeout = cmd_timeout
        self.reconnect_delay = reconnect_delay
        self.log = logger
        self.conn = None
        self.is_connected = False
        self.consecutive_failures = 0
        self._lock = asyncio.Lock()
        self.last_success = time.time()

    # ------------------------------------------------------------------
    async def connect(self) -> bool:
        async with self._lock:
            try:
                self.log.info(f"Connecting to GO2 at {self.ip} …")
                if self.conn:
                    try:
                        await self.conn.disconnect()
                    except Exception:
                        pass
                    self.conn = None

                self.conn = Go2WebRTCConnection(
                    WebRTCConnectionMethod.LocalSTA, ip=self.ip
                )
                await asyncio.wait_for(self.conn.connect(), timeout=30)
                self.is_connected = True
                self.consecutive_failures = 0
                self.last_success = time.time()
                self.log.info("Connected to GO2 robot ✓")
                return True
            except asyncio.TimeoutError:
                self.log.error("Connection timed out")
                self.is_connected = False
                return False
            except SystemExit:
                self.log.error("Driver attempted sys.exit() — caught")
                self.is_connected = False
                return False
            except Exception as exc:
                self.log.error(f"Connection failed: {exc}")
                self.is_connected = False
                return False

    # ------------------------------------------------------------------
    async def ensure_connected(self) -> bool:
        if self.is_connected and self.conn and self.conn.isConnected:
            return True
        self.is_connected = False
        self.log.warning("Connection lost — reconnecting …")
        while True:
            if await self.connect():
                return True
            self.log.info(f"Retrying in {self.reconnect_delay}s …")
            await asyncio.sleep(self.reconnect_delay)

    # ------------------------------------------------------------------
    def _build_payload(self, data: dict) -> dict:
        generated_id = int(time.time() * 1000) % 2_147_483_648
        payload = {
            "header": {
                "identity": {
                    "id": data.get("id", generated_id),
                    "api_id": data.get("api_id", 0),
                }
            },
            "parameter": "",
        }
        if "parameter" in data:
            param = data["parameter"]
            payload["parameter"] = param if isinstance(param, str) else json.dumps(param)
        return payload

    # ------------------------------------------------------------------
    async def send_command(self, topic: str, data: dict) -> bool:
        if not await self.ensure_connected():
            return False
        try:
            self.conn.datachannel.pub_sub.publish_without_callback(
                topic, self._build_payload(data)
            )
            self.last_success = time.time()
            self.consecutive_failures = 0
            return True
        except Exception as exc:
            self.consecutive_failures += 1
            self.log.warning(f"Command failed: {exc}")
            if self.consecutive_failures >= 3:
                self.is_connected = False
            return False

    # ------------------------------------------------------------------
    async def disconnect(self):
        if self.conn:
            try:
                await self.conn.disconnect()
            except Exception:
                pass
        self.conn = None
        self.is_connected = False
        self.log.info("Disconnected from GO2")


# ---------------------------------------------------------------------------
# Pending service request helpers
# ---------------------------------------------------------------------------
class _PendingRequest:
    """Carries a service request into the async loop and reports back."""

    def __init__(self, kind: str, payload: dict):
        self.kind = kind          # 'led' | 'gait' | 'pose'
        self.payload = payload    # kind-specific dict
        self._future: concurrent.futures.Future = concurrent.futures.Future()

    def set_result(self, success: bool, message: str):
        if not self._future.done():
            self._future.set_result((success, message))

    def wait(self, timeout: float = 10.0):
        """Block the ROS service thread until the async loop resolves this."""
        try:
            return self._future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            return False, "Timed out waiting for WebRTC response"


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
class CmdVelBridgeNode(Node):
    """
    Subscribes to /cmd_vel and forwards commands to the GO2 via WebRTC.

    Mapping
    -------
    linear.x  → forward/backward speed  (x in GO2 Move API)
    linear.y  → lateral speed           (y in GO2 Move API)
    angular.z → yaw rate                (z in GO2 Move API)
    """

    # LED colour constants (index → VUI_COLOR)
    _VUI_COLORS = [
        "WHITE", "RED", "YELLOW", "BLUE", "GREEN", "CYAN", "PURPLE"
    ]

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('cmd_timeout', 5.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('max_speed', 1.0)

        self._robot_ip       = self.get_parameter('robot_ip').get_parameter_value().string_value
        self._cmd_timeout    = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self._reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        self._max_speed      = self.get_parameter('max_speed').get_parameter_value().double_value

        self.get_logger().info(f"robot_ip        = {self._robot_ip}")
        self.get_logger().info(f"cmd_timeout     = {self._cmd_timeout}s")
        self.get_logger().info(f"reconnect_delay = {self._reconnect_delay}s")
        self.get_logger().info(f"max_speed       = {self._max_speed}")

        if not GO2_DRIVER_AVAILABLE:
            self.get_logger().fatal(
                f"go2_webrtc_driver is NOT installed: {_IMPORT_ERROR}\n"
                "Install it with:  pip install -e "
                "/path/to/go2_webrtc_connect/"
            )

        # ---- Velocity state ----------------------------------------------
        self._x_speed = 0.0
        self._y_speed = 0.0
        self._z_speed = 0.0
        self._cmd_lock = threading.Lock()
        self._running = True

        # ---- Gait state (tracked live from robot) ------------------------
        self._current_gait = -1
        self._target_gait  = -1
        self._gait_lock    = threading.Lock()

        # ---- Pending one-shot service requests (thread-safe queue) -------
        self._pending_lock    = threading.Lock()
        self._pending_request: _PendingRequest | None = None

        # ---- Publishers / Subscribers / Services -------------------------
        self._connected_pub = self.create_publisher(Bool, 'go2_bridge/connected', 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )

        # Proper typed services
        self._led_srv  = self.create_service(SetLed,  'set_led',  self._set_led_callback)
        self._gait_srv = self.create_service(SetGait, 'set_gait', self._set_gait_callback)
        self._pose_srv = self.create_service(SetPose, 'set_pose', self._set_pose_callback)

        # ---- Asyncio event loop in background thread ---------------------
        self._loop = asyncio.new_event_loop()
        self._bg_thread = threading.Thread(
            target=self._run_event_loop, daemon=True, name='go2_webrtc_loop'
        )
        self._bg_thread.start()

        # ---- Status timer (publishes /go2_bridge/connected at 1 Hz) ------
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            "cmd_vel_bridge node started — services: /set_led  /set_gait  /set_pose"
        )

    # ------------------------------------------------------------------
    # Helpers — queue a request and block until resolved
    # ------------------------------------------------------------------
    def _submit_request(self, kind: str, payload: dict, timeout: float = 10.0):
        """
        Submit a service request to the async loop and block until done.
        Returns (success: bool, message: str).
        """
        req = _PendingRequest(kind, payload)
        with self._pending_lock:
            self._pending_request = req
        return req.wait(timeout=timeout)

    # ------------------------------------------------------------------
    # ROS service callbacks
    # ------------------------------------------------------------------
    def _set_led_callback(self, request: SetLed.Request, response: SetLed.Response):
        """
        /set_led — Change the VUI LED colour.
        color_index: 0=WHITE  1=RED  2=YELLOW  3=BLUE  4=GREEN  5=CYAN  6=PURPLE
        """
        idx = request.color_index
        if not (0 <= idx < len(self._VUI_COLORS)):
            response.success = False
            response.message = (
                f"Invalid color_index {idx}. "
                f"Valid range: 0–{len(self._VUI_COLORS) - 1} "
                f"({', '.join(self._VUI_COLORS)})"
            )
            self.get_logger().warning(response.message)
            return response

        self.get_logger().info(f"[set_led] Requesting colour index {idx} ({self._VUI_COLORS[idx]})")
        success, msg = self._submit_request('led', {'color_index': idx})
        response.success = success
        response.message = msg
        return response

    def _set_gait_callback(self, request: SetGait.Request, response: SetGait.Response):
        """
        /set_gait — Switch locomotion gait.
        gait_type: 0=Idle  1=Trot  2=Run  3=Stair  4=Sport
        """
        gait = request.gait_type
        if not (0 <= gait <= 4):
            response.success = False
            response.message = f"Invalid gait_type {gait}. Valid range: 0–4"
            self.get_logger().warning(response.message)
            return response

        self.get_logger().info(f"[set_gait] Requesting gait type {gait}")
        with self._gait_lock:
            self._target_gait = gait

        success, msg = self._submit_request('gait', {'gait_type': gait})
        response.success = success
        response.message = msg
        return response

    def _set_pose_callback(self, request: SetPose.Request, response: SetPose.Response):
        """
        /set_pose — StandUp (stand_up=true) or StandDown (stand_up=false).
        """
        action = "StandUp" if request.stand_up else "StandDown"
        self.get_logger().info(f"[set_pose] Requesting {action}")
        success, msg = self._submit_request('pose', {'action': action})
        response.success = success
        response.message = msg
        return response

    # ------------------------------------------------------------------
    # /cmd_vel callback
    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist):
        """Store latest velocity command (thread-safe)."""
        with self._cmd_lock:
            self._x_speed = max(-self._max_speed, min(self._max_speed, msg.linear.x))
            self._y_speed = max(-self._max_speed, min(self._max_speed, msg.linear.y))
            self._z_speed = max(-self._max_speed, min(self._max_speed, msg.angular.z))
        self.get_logger().debug(
            f"cmd_vel → x={self._x_speed:.3f}  y={self._y_speed:.3f}  z={self._z_speed:.3f}"
        )

    def _publish_status(self):
        msg = Bool()
        msg.data = self._conn_mgr.is_connected if hasattr(self, '_conn_mgr') else False
        self._connected_pub.publish(msg)

    # ------------------------------------------------------------------
    # Background asyncio loop
    # ------------------------------------------------------------------
    def _run_event_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._command_loop())
        except Exception as exc:
            self.get_logger().error(f"Event loop crashed: {exc}")

    # ------------------------------------------------------------------
    async def _dispatch_pending(self, conn_mgr: ConnectionManager):
        """
        Check if there is a pending one-shot service request and dispatch
        it over WebRTC.  Resolves the request's future so the service
        callback thread can return the result.
        """
        with self._pending_lock:
            req = self._pending_request
            self._pending_request = None

        if req is None:
            return

        kind    = req.kind
        payload = req.payload

        try:
            if kind == 'led':
                idx = payload['color_index']
                color_val = VUI_COLOR.__dict__.get(self._VUI_COLORS[idx], 0)
                ok = await conn_mgr.send_command(
                    RTC_TOPIC["VUI"],
                    {"api_id": 1007, "parameter": {"color": color_val}}
                )
                if ok:
                    req.set_result(True,  f"LED colour set to {self._VUI_COLORS[idx]}")
                else:
                    req.set_result(False, "WebRTC command failed — check connection")

            elif kind == 'gait':
                gait = payload['gait_type']
                ok = await conn_mgr.send_command(
                    RTC_TOPIC["SPORT_MOD"],
                    {"api_id": SPORT_CMD["SwitchGait"], "parameter": {"data": gait}}
                )
                if ok:
                    req.set_result(True,  f"Gait switch to type {gait} dispatched")
                else:
                    req.set_result(False, "WebRTC gait command failed")

            elif kind == 'pose':
                action = payload['action']
                ok = await conn_mgr.send_command(
                    RTC_TOPIC["SPORT_MOD"],
                    {"api_id": SPORT_CMD[action]}
                )
                if ok:
                    req.set_result(True,  f"Pose command '{action}' dispatched successfully")
                else:
                    req.set_result(False, f"WebRTC pose command '{action}' failed")

            else:
                req.set_result(False, f"Unknown request kind: {kind}")

        except Exception as exc:
            req.set_result(False, f"Exception dispatching {kind}: {exc}")

    # ------------------------------------------------------------------
    async def _command_loop(self):
        """
        Continuously reads the latest cmd_vel and sends commands to the robot.
        Runs at ~50 Hz.  One-shot service requests are dispatched inline.
        """
        if not GO2_DRIVER_AVAILABLE:
            self.get_logger().error("go2_webrtc_driver unavailable — command loop idle")
            while self._running:
                await asyncio.sleep(1.0)
            return

        self._conn_mgr = ConnectionManager(
            ip=self._robot_ip,
            cmd_timeout=self._cmd_timeout,
            reconnect_delay=self._reconnect_delay,
            logger=self.get_logger(),
        )

        # Initial connection (retries indefinitely)
        await self._conn_mgr.ensure_connected()
        self.get_logger().info("Command loop running at 50 Hz")

        # Subscribe to sportmodestat to track live gait state
        def sportmodestatus_callback(message):
            try:
                gait = message['data']['gait_type']
                with self._gait_lock:
                    self._current_gait = gait
            except (KeyError, TypeError):
                pass

        if self._conn_mgr.conn and self._conn_mgr.conn.datachannel:
            self._conn_mgr.conn.datachannel.pub_sub.subscribe(
                RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback
            )

        while self._running:
            try:
                # --- Dispatch any pending one-shot service request first ---
                await self._dispatch_pending(self._conn_mgr)

                # --- Gait switching (continuous until matched) -------------
                with self._gait_lock:
                    current_gait = self._current_gait
                    target_gait  = self._target_gait

                if target_gait != -1 and current_gait != target_gait:
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {"api_id": SPORT_CMD["SwitchGait"], "parameter": {"data": target_gait}}
                    )
                    await asyncio.sleep(0.5)
                    continue

                # --- Velocity command ---------------------------------------
                with self._cmd_lock:
                    x = self._x_speed
                    y = self._y_speed
                    z = self._z_speed

                dead = 0.01
                if abs(x) < dead and abs(y) < dead and abs(z) < dead:
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {"api_id": SPORT_CMD["StopMove"]},
                    )
                else:
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": x, "y": y, "z": z},
                        },
                    )

                await asyncio.sleep(0.02)   # 50 Hz

            except Exception as exc:
                self.get_logger().error(f"Command loop error: {exc}")
                self._conn_mgr.is_connected = False
                await asyncio.sleep(1.0)

                if await self._conn_mgr.ensure_connected() \
                        and self._conn_mgr.conn \
                        and self._conn_mgr.conn.datachannel:
                    self._conn_mgr.conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback
                    )

        await self._conn_mgr.disconnect()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down cmd_vel_bridge …")
        self._running = False
        self._bg_thread.join(timeout=3.0)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
