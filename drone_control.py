#!/usr/bin/env python3
"""
Complete MAVSDK Drone Control Script - FIXED for latest version
Handles all acknowledgments properly with clear status reporting

NOTE ON THE FIX:
Newer versions of mavsdk-python changed how action commands report
success/failure. Methods like drone.action.arm(), .takeoff(), .land(), etc.
no longer return an ActionResult you can compare to ActionResult.SUCCESS.
Instead:
    - On success: the coroutine returns None
    - On failure: it raises mavsdk.action.ActionError
So every action call below is wrapped in try/except ActionError instead of
checking a return value.
"""

import asyncio
import sys
from enum import Enum
from mavsdk import System
from mavsdk.action import ActionError

class DroneStatus(Enum):
    """Drone status states"""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ARMED = "armed"
    DISARMED = "disarmed"
    FLYING = "flying"
    LANDED = "landed"

class DroneController:
    """Main drone controller with proper ACK handling"""

    def __init__(self, connection_string="udpin://0.0.0.0:14540"):
        self.drone = System()
        self.connection_string = connection_string
        self.status = DroneStatus.DISCONNECTED
        self._command_timeout = 10  # seconds

    async def connect(self, timeout=10):
        """Connect to drone/simulator with timeout"""
        print(f"🔌 Connecting to {self.connection_string}...")

        try:
            await self.drone.connect(system_address=self.connection_string)

            # Wait for connection with timeout
            connected = False
            for _ in range(timeout * 10):  # 0.1s intervals
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        connected = True
                        break
                if connected:
                    break
                await asyncio.sleep(0.1)

            if not connected:
                print("❌ Connection timeout!")
                return False

            self.status = DroneStatus.CONNECTED
            print("✅ Connected successfully!")

            # Get system info
            try:
                async for version in self.drone.core.version():
                    print(f"   📡 Version: {version}")
                    break
            except Exception:
                pass

            return True

        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False

    async def arm(self):
        """Arm the drone with proper ACK handling"""
        print("🔫 Arming...")

        try:
            await self.drone.action.arm()
            self.status = DroneStatus.ARMED
            print("✅ Armed successfully!")
            return True
        except ActionError as e:
            print(f"❌ Arm failed: {e}")
            return False
        except Exception as e:
            print(f"❌ Arm error: {e}")
            return False

    async def disarm(self):
        """Disarm the drone"""
        print("🔫 Disarming...")

        try:
            await self.drone.action.disarm()
            self.status = DroneStatus.DISARMED
            print("✅ Disarmed successfully!")
            return True
        except ActionError as e:
            print(f"❌ Disarm failed: {e}")
            return False
        except Exception as e:
            print(f"❌ Disarm error: {e}")
            return False

    async def takeoff(self, altitude_m=10):
        """Takeoff to specified altitude"""
        print(f"🚁 Taking off to {altitude_m}m...")

        try:
            # Set the takeoff altitude before commanding takeoff, since
            # takeoff() itself doesn't take an altitude argument.
            await self.drone.action.set_takeoff_altitude(altitude_m)
            await self.drone.action.takeoff()
            self.status = DroneStatus.FLYING
            print(f"✅ Took off to {altitude_m}m!")
            return True
        except ActionError as e:
            print(f"❌ Takeoff failed: {e}")
            return False
        except Exception as e:
            print(f"❌ Takeoff error: {e}")
            return False

    async def land(self):
        """Land the drone"""
        print("🛬 Landing...")

        try:
            await self.drone.action.land()
            self.status = DroneStatus.LANDED
            print("✅ Landing initiated!")
            return True
        except ActionError as e:
            print(f"❌ Landing failed: {e}")
            return False
        except Exception as e:
            print(f"❌ Landing error: {e}")
            return False

    async def return_to_launch(self):
        """Return to launch point"""
        print("🔄 Returning to launch...")

        try:
            await self.drone.action.return_to_launch()
            print("✅ Return to launch initiated!")
            return True
        except ActionError as e:
            print(f"❌ Return to launch failed: {e}")
            return False
        except Exception as e:
            print(f"❌ Return to launch error: {e}")
            return False

    async def get_telemetry(self):
        """Get current telemetry data"""
        try:
            # Get position
            async for pos in self.drone.telemetry.position():
                print(f"   📍 Position: {pos.latitude_deg:.6f}, {pos.longitude_deg:.6f}, {pos.relative_altitude_m:.1f}m")
                break

            # Get attitude
            async for att in self.drone.telemetry.attitude_quaternion():
                print(f"   🧭 Attitude: q=[{att.w:.2f}, {att.x:.2f}, {att.y:.2f}, {att.z:.2f}]")
                break

            # Get battery
            async for batt in self.drone.telemetry.battery():
                print(f"   🔋 Battery: {batt.remaining_percent:.0f}%")
                break

            # Get flight mode
            async for mode in self.drone.telemetry.flight_mode():
                print(f"   🎯 Flight Mode: {mode}")
                break

            return True

        except Exception as e:
            print(f"❌ Telemetry error: {e}")
            return False

    async def wait_until_landed(self, timeout=30):
        """Poll telemetry until the drone reports it is no longer in the air"""
        print("⏳ Waiting for landing to complete...")
        try:
            elapsed = 0.0
            poll_interval = 0.5
            while elapsed < timeout:
                async for in_air in self.drone.telemetry.in_air():
                    if not in_air:
                        print("✅ Confirmed landed!")
                        return True
                    break
                await asyncio.sleep(poll_interval)
                elapsed += poll_interval
            print("⚠️ Timed out waiting for landed confirmation")
            return False
        except Exception as e:
            print(f"❌ Error waiting for landed state: {e}")
            return False

    async def hover(self, duration_sec=5):
        """Hover for specified duration"""
        print(f"⏳ Hovering for {duration_sec} seconds...")
        try:
            await asyncio.sleep(duration_sec)
            print("✅ Hover complete!")
            return True
        except Exception as e:
            print(f"❌ Hover error: {e}")
            return False

    async def perform_mission(self):
        """Complete mission flow with proper ACK handling"""
        print("\n" + "="*50)
        print("🚁 STARTING MISSION")
        print("="*50 + "\n")

        # Step 1: Connect
        if not await self.connect():
            return False

        # Step 2: Arm
        if not await self.arm():
            return False

        # Step 3: Takeoff
        if not await self.takeoff(altitude_m=10):
            await self.disarm()
            return False

        # Step 4: Get telemetry
        print("\n📊 Telemetry:")
        await self.get_telemetry()

        # Step 5: Hover
        print("\n⏳ Hovering...")
        await self.hover(5)

        # Step 6: Land
        print()
        if not await self.land():
            print("⚠️ Landing issue, attempting emergency land...")
            try:
                await self.drone.action.kill()
                print("💀 Emergency kill executed!")
            except Exception:
                pass
            return False

        # Step 7: Wait for the drone to actually finish landing, then disarm
        await self.wait_until_landed()
        await self.disarm()

        print("\n" + "="*50)
        print("✅ MISSION COMPLETE!")
        print("="*50)
        return True

# ============= COMMAND LINE INTERFACE =============

async def main():
    """Command-line interface"""
    import argparse

    parser = argparse.ArgumentParser(description='MAVSDK Drone Controller')
    parser.add_argument('command', nargs='?', default='mission',
                       choices=['mission', 'connect', 'arm', 'disarm', 'takeoff', 'land', 'rtl', 'telemetry'],
                       help='Command to execute')
    parser.add_argument('--altitude', type=int, default=10,
                       help='Takeoff altitude in meters (default: 10)')
    parser.add_argument('--timeout', type=int, default=10,
                       help='Command timeout in seconds (default: 10)')
    parser.add_argument('--connection', default='udpin://0.0.0.0:14540',
                       help='Connection string (default: udpin://0.0.0.0:14540)')

    args = parser.parse_args()

    # Create controller
    controller = DroneController(connection_string=args.connection)
    controller._command_timeout = args.timeout

    # Execute command
    if args.command == 'mission':
        await controller.perform_mission()

    elif args.command == 'connect':
        await controller.connect()

    elif args.command == 'arm':
        await controller.connect()
        await controller.arm()

    elif args.command == 'disarm':
        await controller.connect()
        await controller.disarm()

    elif args.command == 'takeoff':
        await controller.connect()
        await controller.arm()
        await controller.takeoff(altitude_m=args.altitude)

    elif args.command == 'land':
        await controller.connect()
        await controller.land()

    elif args.command == 'rtl':
        await controller.connect()
        await controller.return_to_launch()

    elif args.command == 'telemetry':
        await controller.connect()
        await controller.get_telemetry()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⚠️ Interrupted by user!")
        sys.exit(0)