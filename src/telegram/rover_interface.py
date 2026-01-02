"""
Rover Interface - HTTP Client for CLOVER Rover Service.

Provides a unified interface to control the rover through the
central RoverService HTTP API. This allows Telegram bot and other
clients to share the same Arduino connection without conflicts.
"""

import asyncio
import aiohttp
from typing import Optional, Dict, Any
from enum import Enum
from dataclasses import dataclass
from loguru import logger


class RoverMode(Enum):
    """Rover operating modes."""
    DISABLED = "disabled"
    MANUAL = "manual"
    TELEOP = "teleop"
    AUTONOMOUS = "autonomous"
    ESTOP = "estop"


@dataclass
class RoverStatus:
    """Current rover status."""
    connected: bool
    mode: RoverMode
    battery_voltage: float
    battery_percentage: float
    emergency_stop: bool
    motor_speeds: Dict[str, int]
    encoder_rpms: Dict[str, float]
    error: Optional[str] = None

    def to_message(self) -> str:
        """Format status as a Telegram message."""
        if not self.connected:
            return "🔴 *Rover Disconnesso*\n\nImpossibile comunicare con Arduino.\nAvvia il RoverService con `start_rover_service.py`"

        # Battery emoji based on level
        if self.battery_percentage > 75:
            bat_emoji = "🔋"
        elif self.battery_percentage > 50:
            bat_emoji = "🔋"
        elif self.battery_percentage > 25:
            bat_emoji = "🪫"
        else:
            bat_emoji = "⚠️"

        # Mode emoji
        mode_emojis = {
            RoverMode.DISABLED: "⏹️",
            RoverMode.MANUAL: "🎮",
            RoverMode.TELEOP: "🥽",
            RoverMode.AUTONOMOUS: "🤖",
            RoverMode.ESTOP: "🚨"
        }
        mode_emoji = mode_emojis.get(self.mode, "❓")

        # Emergency stop status
        estop = "🚨 *EMERGENCY STOP ATTIVO*\n\n" if self.emergency_stop else ""

        msg = f"{estop}📊 *Stato CLOVER Rover*\n\n"
        msg += f"{mode_emoji} *Modalità:* {self.mode.value.upper()}\n"
        msg += f"{bat_emoji} *Batteria:* {self.battery_voltage:.2f}V ({self.battery_percentage:.0f}%)\n\n"

        msg += "🔧 *Motori (PWM):*\n"
        msg += f"  FL: {self.motor_speeds.get('FL', 0):+4d}  FR: {self.motor_speeds.get('FR', 0):+4d}\n"
        msg += f"  RL: {self.motor_speeds.get('RL', 0):+4d}  RR: {self.motor_speeds.get('RR', 0):+4d}\n\n"

        msg += "📈 *Encoder (RPM):*\n"
        msg += f"  FL: {self.encoder_rpms.get('FL', 0):+6.1f}  FR: {self.encoder_rpms.get('FR', 0):+6.1f}\n"
        msg += f"  RL: {self.encoder_rpms.get('RL', 0):+6.1f}  RR: {self.encoder_rpms.get('RR', 0):+6.1f}"

        return msg


class RoverInterface:
    """
    HTTP Client for CLOVER Rover Service.

    Connects to the central RoverService to control the rover.
    Multiple clients can use this interface simultaneously.
    """

    # Battery thresholds (3S LiPo)
    BATTERY_MIN = 9.0
    BATTERY_MAX = 12.6
    BATTERY_WARNING = 10.0

    def __init__(
        self,
        service_url: str = "http://localhost:8081",
        timeout: float = 5.0
    ):
        """
        Initialize rover interface.

        Args:
            service_url: URL of the RoverService (default: http://localhost:8081)
            timeout: HTTP request timeout in seconds
        """
        self.service_url = service_url.rstrip('/')
        self.timeout = aiohttp.ClientTimeout(total=timeout)
        self._session: Optional[aiohttp.ClientSession] = None
        self._current_mode = RoverMode.DISABLED

        logger.info(f"RoverInterface initialized (service={service_url})")

    async def _get_session(self) -> aiohttp.ClientSession:
        """Get or create HTTP session."""
        if self._session is None or self._session.closed:
            self._session = aiohttp.ClientSession(timeout=self.timeout)
        return self._session

    async def close(self):
        """Close HTTP session."""
        if self._session and not self._session.closed:
            await self._session.close()

    async def _request(self, method: str, endpoint: str, data: dict = None) -> dict:
        """Make HTTP request to RoverService."""
        session = await self._get_session()
        url = f"{self.service_url}{endpoint}"

        try:
            if method == "GET":
                async with session.get(url) as response:
                    return await response.json()
            elif method == "POST":
                async with session.post(url, json=data or {}) as response:
                    return await response.json()
        except aiohttp.ClientError as e:
            logger.error(f"HTTP error: {e}")
            return {"success": False, "error": str(e)}
        except Exception as e:
            logger.error(f"Request error: {e}")
            return {"success": False, "error": str(e)}

    # ==================== Connection ====================

    async def connect(self, port: str = "/dev/ttyUSB0") -> bool:
        """Connect to Arduino through RoverService."""
        result = await self._request("POST", "/connect", {"port": port})
        success = result.get("success", False)
        if success:
            logger.info(f"Connected to Arduino via RoverService")
        else:
            logger.error(f"Connection failed: {result.get('error')}")
        return success

    async def disconnect(self) -> bool:
        """Disconnect from Arduino."""
        result = await self._request("POST", "/disconnect")
        logger.info("Disconnected from Arduino")
        return result.get("success", False)

    @property
    def is_connected(self) -> bool:
        """Check if connected (sync check - use get_status for accurate state)."""
        return True  # Service manages connection

    async def check_service(self) -> bool:
        """Check if RoverService is running."""
        try:
            result = await self._request("GET", "/health")
            return result.get("status") == "ok"
        except:
            return False

    # ==================== Status ====================

    async def get_status(self) -> RoverStatus:
        """Get current rover status from RoverService."""
        try:
            result = await self._request("GET", "/status")

            if "error" in result and not result.get("connected", False):
                return RoverStatus(
                    connected=False,
                    mode=RoverMode.DISABLED,
                    battery_voltage=0.0,
                    battery_percentage=0.0,
                    emergency_stop=False,
                    motor_speeds={},
                    encoder_rpms={},
                    error=result.get("error", "Service not available")
                )

            # Parse mode
            mode_str = result.get("mode", "disabled")
            try:
                mode = RoverMode(mode_str)
            except ValueError:
                mode = RoverMode.DISABLED

            self._current_mode = mode

            return RoverStatus(
                connected=result.get("connected", False),
                mode=mode,
                battery_voltage=result.get("battery_voltage", 0.0),
                battery_percentage=result.get("battery_percentage", 0.0),
                emergency_stop=result.get("emergency_stop", False),
                motor_speeds=result.get("motor_speeds", {}),
                encoder_rpms=result.get("encoder_rpms", {}),
                error=result.get("error")
            )

        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return RoverStatus(
                connected=False,
                mode=RoverMode.DISABLED,
                battery_voltage=0.0,
                battery_percentage=0.0,
                emergency_stop=False,
                motor_speeds={},
                encoder_rpms={},
                error=str(e)
            )

    # ==================== Mode Control ====================

    async def set_mode(self, mode: RoverMode) -> bool:
        """Set rover operating mode."""
        result = await self._request("POST", "/mode", {"mode": mode.value})
        success = result.get("success", False)
        if success:
            self._current_mode = mode
            logger.info(f"Mode set to: {mode.value}")
        return success

    # ==================== Motor Control ====================

    async def emergency_stop(self) -> bool:
        """Trigger emergency stop."""
        result = await self._request("POST", "/estop")
        if result.get("success"):
            self._current_mode = RoverMode.ESTOP
            logger.warning("EMERGENCY STOP triggered")
        return result.get("success", False)

    async def release_emergency_stop(self) -> bool:
        """Release emergency stop."""
        result = await self._request("POST", "/release")
        if result.get("success"):
            self._current_mode = RoverMode.DISABLED
            logger.info("Emergency stop released")
        return result.get("success", False)

    async def stop_motors(self) -> bool:
        """Stop all motors (soft stop)."""
        result = await self._request("POST", "/stop")
        if result.get("success"):
            logger.info("Motors stopped")
        return result.get("success", False)

    async def move(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> bool:
        """
        Move rover using velocity commands.

        Args:
            vx: Forward velocity (m/s), positive = forward
            vy: Lateral velocity (m/s), positive = left
            wz: Angular velocity (rad/s), positive = counter-clockwise
        """
        result = await self._request("POST", "/move", {
            "vx": vx,
            "vy": vy,
            "wz": wz
        })
        return result.get("success", False)

    # ==================== Convenience Methods ====================

    async def move_forward(self, speed: float = 0.3) -> bool:
        """Move forward at specified speed (m/s)."""
        return await self.move(vx=speed)

    async def move_backward(self, speed: float = 0.3) -> bool:
        """Move backward at specified speed (m/s)."""
        return await self.move(vx=-speed)

    async def strafe_left(self, speed: float = 0.3) -> bool:
        """Strafe left at specified speed (m/s)."""
        return await self.move(vy=speed)

    async def strafe_right(self, speed: float = 0.3) -> bool:
        """Strafe right at specified speed (m/s)."""
        return await self.move(vy=-speed)

    async def rotate_left(self, speed: float = 1.0) -> bool:
        """Rotate counter-clockwise at specified speed (rad/s)."""
        return await self.move(wz=speed)

    async def rotate_right(self, speed: float = 1.0) -> bool:
        """Rotate clockwise at specified speed (rad/s)."""
        return await self.move(wz=-speed)

    # ==================== Battery ====================

    async def check_battery_warning(self) -> Optional[str]:
        """Check if battery voltage is low and return warning message."""
        status = await self.get_status()
        if status.connected and status.battery_voltage > 0:
            if status.battery_voltage <= self.BATTERY_MIN:
                return f"🚨 *BATTERIA CRITICA!* {status.battery_voltage:.2f}V - SPEGNERE IMMEDIATAMENTE!"
            elif status.battery_voltage <= self.BATTERY_WARNING:
                return f"⚠️ *Batteria bassa:* {status.battery_voltage:.2f}V ({status.battery_percentage:.0f}%)"
        return None
