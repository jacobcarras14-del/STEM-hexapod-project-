# ============================================================================
# HEXAPOD LEG CONTROLLER - Real Robot (Windows PC, COM4)
# ============================================================================

import serial
import time
import math

# ============================================================================
# IK SOLVER
# ============================================================================

class LegIK:
    """Inverse kinematics for 3-joint hexapod leg"""
    
    def __init__(self, coxa_length=0.05, femur_length=0.10, tibia_length=0.17):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
    
    def calculate(self, target_x, target_y, target_z):
        """
        Calculate joint angles for target foot position
        Returns (coxa, femur, tibia) in radians, or None if unreachable
        """
        # Coxa angle
        coxa_angle = math.atan2(target_y, target_x)
        
        # Horizontal distance
        horizontal_dist = math.sqrt(target_x**2 + target_y**2)
        femur_to_target_horiz = horizontal_dist - self.coxa_length
        femur_to_target_3d = math.sqrt(femur_to_target_horiz**2 + target_z**2)
        
        # Check reachability
        max_reach = self.femur_length + self.tibia_length
        min_reach = abs(self.femur_length - self.tibia_length)
        
        if femur_to_target_3d > max_reach or femur_to_target_3d < min_reach:
            print(f"Warning: Unreachable (dist={femur_to_target_3d:.3f})")
            return None
        
        # Tibia angle (law of cosines)
        cos_tibia = (self.femur_length**2 + self.tibia_length**2 - femur_to_target_3d**2) / \
                    (2 * self.femur_length * self.tibia_length)
        cos_tibia = max(-1.0, min(1.0, cos_tibia))
        tibia_interior = math.acos(cos_tibia)
        tibia_angle = -(math.pi - tibia_interior)
        
        # Femur angle (law of cosines)
        cos_femur = (self.femur_length**2 + femur_to_target_3d**2 - self.tibia_length**2) / \
                    (2 * self.femur_length * femur_to_target_3d)
        cos_femur = max(-1.0, min(1.0, cos_femur))
        femur_triangle = math.acos(cos_femur)
        
        elevation = math.atan2(-target_z, femur_to_target_horiz)
        femur_angle = elevation + femur_triangle
        
        return (coxa_angle, femur_angle, tibia_angle)


# ============================================================================
# HEXAPOD LEG CONTROLLER
# ============================================================================

class HexapodLegReal:
    """Controls real hexapod leg via serial"""
    
    def __init__(self, serial_port='COM4', leg_number=1, mounting_angle=0.0):
        """
        Args:
            serial_port: COM port (e.g., 'COM4')
            leg_number: 1-6
            mounting_angle: Radians (e.g., math.radians(33))
        """
        self.leg_number = leg_number
        self.mounting_angle = mounting_angle
        
        # Connect to Arduino
        print(f"Connecting to Arduino on {serial_port}...")
        try:
            self.serial = serial.Serial(serial_port, 115200, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            print("Connected!")
        except Exception as e:
            print(f"ERROR: Could not connect to {serial_port}")
            print(f"Error: {e}")
            print("\nTroubleshooting:")
            print("1. Check if Arduino is plugged in")
            print("2. Verify COM port in Device Manager")
            print("3. Close Arduino IDE Serial Monitor")
            raise
        
        # Create IK solver
        self.ik = LegIK(coxa_length=0.05, femur_length=0.10, tibia_length=0.17)
        
        # Initial foot position (from simulation)
        self.initial_pos = [-0.131, -0.064, -0.096]
        
        # Gait state
        self.step_progression = 0.0
        self.real_movement_strength = 0.0
        
        # Movement parameters
        self.mov_data = {
            'vel': 0.5,
            'amplitude': 0.16,
            'height': 0.04,
            'dir': 0.0,
            'rot': 0.0,
            'strength': 0.0
        }
        
        print(f"Leg {leg_number} initialized at {math.degrees(mounting_angle):.1f} degrees")
    
    def set_movement_params(self, velocity=0.5, amplitude=0.16, height=0.04,
                           direction=0.0, rotation=0.0, strength=1.0):
        """
        Set movement parameters
        
        Args:
            velocity: Step speed (0.5-1.5 typical)
            amplitude: Step size in meters (0.05-0.20)
            height: Lift height in meters (0.02-0.08)
            direction: Direction in degrees (0=forward, 90=right, 180=back, 270=left)
            rotation: Rotation (-1=CCW, 0=none, 1=CW)
            strength: Movement strength (0=stop, 1=full speed)
        """
        self.mov_data['vel'] = velocity
        self.mov_data['amplitude'] = amplitude
        self.mov_data['height'] = height
        self.mov_data['dir'] = math.radians(direction)
        self.mov_data['rot'] = rotation
        self.mov_data['strength'] = strength
    
    def calculate_foot_offset(self, step_phase):
        """Calculate foot offset for gait phase (same as simulation)"""
        sp = step_phase % 1.0
        offset = [0.0, 0.0, 0.0]
        
        if sp < (1/3):
            offset[0] = sp * 3 * self.mov_data['amplitude'] / 2
        elif sp < (1/3 + 1/6):
            s = sp - 1/3
            offset[0] = self.mov_data['amplitude']/2 - self.mov_data['amplitude']*s*6/2
            offset[2] = s * 6 * self.mov_data['height']
        elif sp < (2/3):
            s = sp - 1/3 - 1/6
            offset[0] = -self.mov_data['amplitude'] * s * 6 / 2
            offset[2] = (1 - s*6) * self.mov_data['height']
        else:
            s = sp - 2/3
            offset[0] = -self.mov_data['amplitude'] * (1 - s*3) / 2
        
        return offset
    
    def transform_offset_to_world(self, offset):
        """Transform to world coordinates (same as simulation)"""
        md = self.mov_data['dir'] + abs(self.mov_data['rot']) * math.atan2(
            self.initial_pos[0] * self.mov_data['rot'],
            -self.initial_pos[1] * self.mov_data['rot']
        )
        
        offset_world = [
            offset[0] * math.sin(md) * self.real_movement_strength,
            offset[0] * math.cos(md) * self.real_movement_strength,
            offset[2] * self.real_movement_strength
        ]
        
        return offset_world
    
    def update(self, dt):
        """Update leg position"""
        # Smooth ramping
        dx = self.mov_data['strength'] - self.real_movement_strength
        if abs(dx) > dt * 0.1:
            dx = abs(dx) * dt * 0.5 / dx
        self.real_movement_strength += dx
        
        # Calculate target foot position
        offset_local = self.calculate_foot_offset(self.step_progression)
        offset_world = self.transform_offset_to_world(offset_local)
        
        target_pos = [
            self.initial_pos[0] + offset_world[0],
            self.initial_pos[1] + offset_world[1],
            self.initial_pos[2] + offset_world[2]
        ]
        
        # Calculate IK
        angles = self.ik.calculate(target_pos[0], target_pos[1], target_pos[2])
        
        if angles:
            self.send_to_arduino(angles)
        
        # Advance step
        self.step_progression += dt * self.mov_data['vel']
    
    def send_to_arduino(self, angles):
        """Send joint angles to Arduino"""
        coxa_deg = math.degrees(angles[0])
        femur_deg = math.degrees(angles[1])
        tibia_deg = math.degrees(angles[2])
        
        command = f"LEG,{self.leg_number},{coxa_deg:.2f},{femur_deg:.2f},{tibia_deg:.2f}\n"
        self.serial.write(command.encode())
    
    def close(self):
        """Close serial connection"""
        if self.serial:
            self.serial.close()


# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    print("="*70)
    print("HEXAPOD LEG CONTROLLER - REAL ROBOT")
    print("Windows PC - COM4 - 270-Degree Servos")
    print("="*70)
    
    try:
        # Create leg controller
        leg = HexapodLegReal(
            serial_port='COM4',  # Your Arduino port
            leg_number=1,
            mounting_angle=math.radians(33)
        )
        
        print("\n--- Press Ctrl+C to stop ---\n")
        
        # Test 1: Walk forward
        print("TEST 1: Walking forward for 10 seconds...")
        leg.set_movement_params(
            velocity=0.9,
            amplitude=0.11,
            height=0.02,
            direction=0,      # Forward
            strength=1.0
        )
        
        start_time = time.time()
        dt = 0.05  # 50ms update rate
        
        while (time.time() - start_time) < 10.0:
            leg.update(dt)
            time.sleep(dt)
        
        # Test 2: Stop
        print("\nTEST 2: Stopping...")
        leg.set_movement_params(strength=0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < 2.0:
            leg.update(dt)
            time.sleep(dt)
        
        # Test 3: Strafe right
        print("\nTEST 3: Strafing right for 5 seconds...")
        leg.set_movement_params(
            direction=90,     # Right
            strength=1.0
        )
        
        start_time = time.time()
        while (time.time() - start_time) < 5.0:
            leg.update(dt)
            time.sleep(dt)
        
        # Stop
        print("\nStopping and closing...")
        leg.set_movement_params(strength=0.0)
        
        start_time = time.time()
        while (time.time() - start_time) < 2.0:
            leg.update(dt)
            time.sleep(dt)
        
        leg.close()
        print("\nDone!")
        
    except KeyboardInterrupt:
        print("\n\nStopped by user (Ctrl+C)")
        leg.close()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
