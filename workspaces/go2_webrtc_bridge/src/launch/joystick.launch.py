import pygame
from pygame.locals import *
import asyncio
import logging
import time

from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, VUI_COLOR

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)    

# =============================================================================
# CONNECTION CONFIGURATION
# =============================================================================
ROBOT_IP = "192.168.8.160"
RECONNECT_DELAY_SECONDS = 3          # Wait time before attempting reconnection
MAX_RECONNECT_ATTEMPTS = 0           # 0 = infinite attempts
HEALTH_CHECK_INTERVAL = 5            # Seconds between connection health checks
COMMAND_TIMEOUT_SECONDS = 5          # Timeout for individual commands (increased from 2)
CONSECUTIVE_FAILURES_BEFORE_RECONNECT = 3  # Number of consecutive failures before triggering reconnect

# =============================================================================
# CONNECTION MANAGER
# =============================================================================
class ConnectionManager:
    """Manages WebRTC connection with automatic reconnection capability."""
    
    def __init__(self, ip: str):
        self.ip = ip
        self.conn = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.last_successful_command = time.time()
        self._connection_lock = asyncio.Lock()
        self.consecutive_failures = 0  # Track consecutive command failures
        
    async def connect(self) -> bool:
        """
        Establish connection to the robot.
        Returns True if connection successful, False otherwise.
        """
        async with self._connection_lock:
            try:
                logger.info(f"🔗 Attempting to connect to robot at {self.ip}...")
                
                # Clean up any existing connection
                if self.conn:
                    try:
                        await self.conn.disconnect()
                    except Exception:
                        pass
                    self.conn = None
                
                # Create new connection
                self.conn = Go2WebRTCConnection(
                    WebRTCConnectionMethod.LocalSTA, 
                    ip=self.ip
                )
                
                # Connect with timeout
                await asyncio.wait_for(
                    self.conn.connect(),
                    timeout=30
                )
                
                self.is_connected = True
                self.reconnect_attempts = 0
                self.consecutive_failures = 0
                self.last_successful_command = time.time()
                logger.info("✅ Successfully connected to robot!")
                
                # Initial LED indication of connection
                await self._set_connected_indicator()
                
                return True
                
            except asyncio.TimeoutError:
                logger.error("❌ Connection timeout - robot not responding")
                self.is_connected = False
                return False
            except SystemExit:
                # Catch sys.exit() calls from the driver - don't let them terminate the program
                logger.error("❌ Connection failed - driver attempted to exit")
                self.is_connected = False
                return False
            except (ConnectionError, Exception) as e:
                logger.error(f"❌ Connection failed: {e}")
                self.is_connected = False
                return False
    
    async def _set_connected_indicator(self):
        """Set LED to indicate successful connection."""
        try:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"], 
                {
                    "api_id": 1007,
                    "parameter": {"color": VUI_COLOR.PURPLE}
                }
            )
        except Exception as e:
            logger.warning(f"Could not set LED indicator: {e}")
    
    async def ensure_connected(self) -> bool:
        """
        Ensure connection is active, reconnect if necessary.
        Returns True if connected, False if all reconnection attempts failed.
        """
        if self.is_connected and self.conn and self.conn.isConnected:
            return True
        
        logger.warning("⚠️ Connection lost, attempting to reconnect...")
        self.is_connected = False
        
        while MAX_RECONNECT_ATTEMPTS == 0 or self.reconnect_attempts < MAX_RECONNECT_ATTEMPTS:
            self.reconnect_attempts += 1
            logger.info(f"🔄 Reconnection attempt {self.reconnect_attempts}...")
            
            if await self.connect():
                return True
            
            logger.info(f"⏳ Waiting {RECONNECT_DELAY_SECONDS} seconds before next attempt...")
            await asyncio.sleep(RECONNECT_DELAY_SECONDS)
        
        logger.error("❌ Max reconnection attempts reached")
        return False
    
    async def send_command(self, topic: str, data: dict, wait_for_response: bool = False) -> bool:
        """
        Send a command to the robot with automatic reconnection on failure.
        
        Args:
            topic: The RTC topic to publish to
            data: The command data
            wait_for_response: If True, wait for response (slower). If False, fire-and-forget (faster).
        
        Returns True if command sent successfully.
        """
        if not await self.ensure_connected():
            return False
        
        try:
            if wait_for_response:
                # Use publish_request_new for commands that need responses
                await asyncio.wait_for(
                    self.conn.datachannel.pub_sub.publish_request_new(topic, data),
                    timeout=COMMAND_TIMEOUT_SECONDS
                )
            else:
                # Use fire-and-forget for movement commands (much faster, no timeout issues)
                self.conn.datachannel.pub_sub.publish_without_callback(
                    topic, 
                    self._build_request_payload(data)
                )
            
            self.last_successful_command = time.time()
            self.consecutive_failures = 0  # Reset failure counter on success
            return True
        except asyncio.TimeoutError:
            self.consecutive_failures += 1
            logger.warning(f"⚠️ Command timeout ({self.consecutive_failures}/{CONSECUTIVE_FAILURES_BEFORE_RECONNECT})")
            if self.consecutive_failures >= CONSECUTIVE_FAILURES_BEFORE_RECONNECT:
                logger.warning("⚠️ Too many consecutive failures - marking connection as lost")
                self.is_connected = False
            return False
        except Exception as e:
            self.consecutive_failures += 1
            logger.warning(f"⚠️ Command failed: {e} ({self.consecutive_failures}/{CONSECUTIVE_FAILURES_BEFORE_RECONNECT})")
            if self.consecutive_failures >= CONSECUTIVE_FAILURES_BEFORE_RECONNECT:
                self.is_connected = False
            return False
    
    def _build_request_payload(self, data: dict) -> dict:
        """Build the request payload for fire-and-forget commands."""
        import json
        generated_id = int(time.time() * 1000) % 2147483648
        
        request_payload = {
            "header": {
                "identity": {
                    "id": data.get("id", generated_id),
                    "api_id": data.get("api_id", 0)
                }
            },
            "parameter": ""
        }
        
        if "parameter" in data:
            param = data["parameter"]
            request_payload["parameter"] = param if isinstance(param, str) else json.dumps(param)
        
        return request_payload
    
    async def check_connection_health(self) -> bool:
        """
        Check if the connection is still healthy.
        """
        if not self.conn or not self.conn.isConnected:
            self.is_connected = False
            return False
        
        # Check if datachannel is still open
        if not self.conn.datachannel or not self.conn.datachannel.data_channel_opened:
            self.is_connected = False
            return False
        
        # If no successful command in a while, connection might be stale
        if time.time() - self.last_successful_command > HEALTH_CHECK_INTERVAL * 2:
            logger.debug("Connection health: checking with StopMove command...")
            # Send a harmless command to verify connection (use wait_for_response=True for health check)
            result = await self.send_command(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["StopMove"]},
                wait_for_response=True
            )
            if result:
                self.consecutive_failures = 0
            return result
        
        return True
    
    async def disconnect(self):
        """Clean disconnect from robot."""
        if self.conn:
            try:
                await self.conn.disconnect()
            except Exception:
                pass
        self.conn = None
        self.is_connected = False
        logger.info("🔌 Disconnected from robot")


# =============================================================================
# PYGAME INITIALIZATION
# =============================================================================
pygame.init()
pygame.joystick.init()



def _find_joystick():
    """Find F710 by name — works regardless of USB index order."""
    count = pygame.joystick.get_count()
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        j.init()
        if 'F710' in j.get_name() or 'Wireless Gamepad' in j.get_name():
            print(f"Found F710 at joystick index {i}: {j.get_name()}")
            return j
    if count > 0:
        j = pygame.joystick.Joystick(0)
        j.init()
        return j
    return None

x_speed = 0
z_speed = 0
y_speed = 0

command_bucket = ""
gait = 0
gait_trigger = False
economic_gait = False
continous_gait = False
speed = 0.5

euler_control = False

height_cm = 25

# VUI LED color cycling
VUI_COLORS = [VUI_COLOR.WHITE, VUI_COLOR.RED, VUI_COLOR.YELLOW, VUI_COLOR.BLUE, VUI_COLOR.GREEN, VUI_COLOR.CYAN, VUI_COLOR.PURPLE]
current_color_index = 0
color_change_requested = False

# Check how many joysticks are connected
joystick_count = pygame.joystick.get_count()
print(f"{joystick_count} joystick(s) detected")

if joystick_count == 0:
    print("No joystick connected.")
    joystick = None
else:
    joystick = _find_joystick()
    print(f"Using joystick: {joystick.get_name()}")

# =============================================================================
# MAIN FUNCTIONS
# =============================================================================
running = True
connection_manager = None

async def send_commands():
    """
    Main command sending loop with automatic reconnection.
    This function runs forever, automatically reconnecting when connection is lost.
    """
    global running
    global command_bucket
    global gait, gait_trigger
    global economic_gait, continous_gait
    global x_speed, z_speed, y_speed
    global euler_control
    global height_cm
    global connection_manager
    global current_color_index, color_change_requested
    
    # Initialize connection manager
    connection_manager = ConnectionManager(ROBOT_IP)
    
    # Initial connection with retry
    while running:
        if await connection_manager.connect():
            break
        logger.info(f"⏳ Retrying initial connection in {RECONNECT_DELAY_SECONDS} seconds...")
        await asyncio.sleep(RECONNECT_DELAY_SECONDS)
    
    if not running:
        return
    
    last_health_check = time.time()
    
    # Main command loop - runs forever with auto-reconnection
    while running:
        try:
            # Periodic health check
            current_time = time.time()
            if current_time - last_health_check > HEALTH_CHECK_INTERVAL:
                if not await connection_manager.check_connection_health():
                    logger.warning("⚠️ Connection health check failed, will reconnect...")
                last_health_check = current_time
            
            # Send joystick data as long as the program is running
            if not euler_control:
                if (int(x_speed*10) == 0 and int(z_speed*10) == 0 and int(y_speed*10) == 0):
                    await connection_manager.send_command(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["StopMove"]}
                    )
                else:
                    await connection_manager.send_command(
                        RTC_TOPIC["SPORT_MOD"], 
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": x_speed, "y": y_speed, "z": z_speed}
                        }
                    )
            else:
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["Euler"],
                        "parameter": {"x": -y_speed, "y": x_speed, "z": z_speed}
                    }
                )
                
                min_cm, max_cm = 20, 35
                min_height_m, max_height_m = -0.18, 0.03
                
                # Clamp height to acceptable range
                clamped_height = max(min_cm, min(height_cm, max_cm))
                
                # Perform linear interpolation from cm to meter range
                height_m = min_height_m + (max_height_m - min_height_m) * ((clamped_height - min_cm) / (max_cm - min_cm))
                        
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["BodyHeight"],
                        "parameter": {"data": height_m}
                    }
                )

            # Handle command bucket (button commands)
            if command_bucket != "":
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {"api_id": SPORT_CMD[command_bucket]}
                )
                await asyncio.sleep(1)
                command_bucket = ""

            # Handle gait switching
            if gait_trigger:
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["SwitchGait"],
                        "parameter": {"data": gait}
                    }
                )
                await asyncio.sleep(1)
                gait_trigger = False

            # Handle economic gait
            if economic_gait:
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["EconomicGait"],
                        "parameter": {"data": 1}
                    }
                )
                await asyncio.sleep(0.5)
                economic_gait = False

                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["ContinuousGait"],
                        "parameter": {"data": 0}
                    }
                )
                await asyncio.sleep(0.5)

            # Handle continuous gait
            if continous_gait:
                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["EconomicGait"],
                        "parameter": {"data": 0}
                    }
                )
                await asyncio.sleep(0.5)

                await connection_manager.send_command(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["ContinuousGait"],
                        "parameter": {"data": 1}
                    }
                )
                await asyncio.sleep(0.5)
                continous_gait = False

            # Handle VUI LED color change
            if color_change_requested:
                await connection_manager.send_command(
                    RTC_TOPIC["VUI"],
                    {
                        "api_id": 1007,
                        "parameter": {"color": VUI_COLORS[current_color_index]}
                    }
                )
                logger.info(f"🎨 VUI LED color changed to: {VUI_COLORS[current_color_index]}")
                await asyncio.sleep(0.2)
                color_change_requested = False

            # Small delay between command cycles
            await asyncio.sleep(0.01)
            
        except Exception as e:
            logger.error(f"❌ Error in command loop: {e}")
            # Mark connection as lost to trigger reconnection
            connection_manager.is_connected = False
            await asyncio.sleep(1)  # Brief pause before retry
    
    # Cleanup on exit
    if connection_manager:
        await connection_manager.disconnect()


# Run the joystick event processing and WebRTC communication in parallel
async def main():
    """
    Main entry point - runs joystick input handling and command sending in parallel.
    The script will run indefinitely, maintaining connection to the robot.
    """
    global running
    global command_bucket
    global gait, gait_trigger
    global economic_gait, continous_gait 
    global x_speed, z_speed, y_speed, speed
    global euler_control
    global height_cm
    global connection_manager
    global current_color_index, color_change_requested
    
    logger.info("=" * 60)
    logger.info("🤖 GO2 Joystick Controller Starting...")
    logger.info("=" * 60)
    logger.info(f"Robot IP: {ROBOT_IP}")
    logger.info(f"Reconnect delay: {RECONNECT_DELAY_SECONDS} seconds")
    logger.info(f"Max reconnect attempts: {'Infinite' if MAX_RECONNECT_ATTEMPTS == 0 else MAX_RECONNECT_ATTEMPTS}")
    logger.info("=" * 60)
    
    # Create a task for sending commands to the WebRTC server
    send_task = asyncio.create_task(send_commands())

    try:
        # Main event loop for joystick input
        while running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                    logger.info("🛑 Quit signal received")

                # Detect joystick button presses or releases
                if event.type == JOYBUTTONDOWN:
                    print(f"Button {event.button} pressed")
                    if event.button == 6:
                        command_bucket = "StandDown"
                    elif event.button == 7:
                        command_bucket = "StandUp"
                    elif event.button == 2:
                        command_bucket = "Stretch"
                    elif event.button == 1:
                        gait = gait+1 if gait != 4 else 0 
                        print(f"gait changed: {gait}")                  
                        gait_trigger = True
                    elif event.button == 3:
                        speed = speed + 0.1
                        print(f"speed changed: {speed}")   
                    elif event.button == 0:
                        speed = speed - 0.1
                        print(f"speed changed: {speed}")   
                    elif event.button == 4:
                        if euler_control:
                            # When Euler control is active, button 4 cycles VUI LED colors
                            current_color_index = (current_color_index + 1) % len(VUI_COLORS)
                            color_change_requested = True
                            print(f"VUI LED color: {VUI_COLORS[current_color_index]}")
                        else:
                            command_bucket = "Hello"

                    elif event.button == 5:
                        euler_control = True
                        print("Euler Control Enabled")
                    
                if event.type == JOYBUTTONUP:
                    print(f"Button {event.button} released")
                    if event.button == 5:   
                        euler_control = False
                        print("Euler Control Disabled")

                # Detect joystick axis movements (analog stick movements)
                if event.type == JOYAXISMOTION and joystick:
                    axis_value = joystick.get_axis(event.axis)
                    # Update x and z speeds based on joystick axis values
                    if event.axis == 3:
                        z_speed = -axis_value * speed / 0.5
                    if event.axis == 1:
                        x_speed = -axis_value * speed / 0.5
                    if event.axis == 0:
                        y_speed = -axis_value * speed / 0.8
                    if event.axis == 4:
                        if axis_value == 0:
                            height_cm = 35
                        else:
                            height_cm = max(21, min(-(axis_value + 0.5) * 15 + 35, 35))
                    
                    # Print the current speed values
                    print(f"x_speed: {x_speed:.2f}, z_speed: {z_speed:.2f}, y_speed: {y_speed:.2f}, height: {height_cm}")
            
            await asyncio.sleep(0.01)  # Small delay to avoid blocking the event loop

    except KeyboardInterrupt:
        logger.info("🛑 Keyboard interrupt received")
        running = False
    except Exception as e:
        logger.error(f"❌ Unexpected error in main loop: {e}")
        running = False
    finally:
        # Cleanup
        running = False
        logger.info("⏳ Waiting for command task to finish...")
        await send_task
        if connection_manager:
            await connection_manager.disconnect()
        logger.info("👋 Goodbye!")


# Run the main loop using asyncio
if __name__ == "__main__":
    while True:  # Outer loop to restart if anything goes wrong
        try:
            asyncio.run(main())
            break  # Normal exit
        except KeyboardInterrupt:
            logger.info("🛑 Script terminated by user")
            break
        except SystemExit as e:
            logger.error(f"🔄 SystemExit caught ({e}), restarting in {RECONNECT_DELAY_SECONDS} seconds...")
            time.sleep(RECONNECT_DELAY_SECONDS)
            # Reset globals for restart
            running = True
            connection_manager = None
        except Exception as e:
            logger.error(f"🔄 Unexpected error ({e}), restarting in {RECONNECT_DELAY_SECONDS} seconds...")
            time.sleep(RECONNECT_DELAY_SECONDS)
            # Reset globals for restart
            running = True
            connection_manager = None
        finally:
            try:
                pygame.quit()
            except:
                pass
            try:
                pygame.init()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    joystick = _find_joystick()
            except:
                pass