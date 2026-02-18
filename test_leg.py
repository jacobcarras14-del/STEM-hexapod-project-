from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

# ============================================================================
# HEXAPOD LEG CONTROLLER - IK-BASED (Like Official Lua Code)
# ============================================================================

class HexapodLegIK:
    """
    Controls hexapod leg using IK (sets foot target positions)
    CoppeliaSim's IK solver calculates joint angles automatically
    """
    
    def __init__(self, sim, simIK, leg_number=1, mounting_angle=0.0):
        """
        Initialize leg with IK setup
        
        Args:
            sim: CoppeliaSim remote API object
            simIK: CoppeliaSim IK module object
            leg_number: Leg number (1-6)
            mounting_angle: Angle in radians (e.g., 33 deg = 0.576 rad)
        """
        self.sim = sim
        self.simIK = simIK
        self.leg_number = leg_number
        self.mounting_angle = mounting_angle
        
        # Get object handles
        self.base = sim.getObject('/base')
        self.leg_base = sim.getObject('/legBase')
        self.foot_tip = sim.getObject(f'/footTip{leg_number}')
        self.foot_target = sim.getObject(f'/footTarget{leg_number}')
        
        # Get joint handles (for monitoring, not direct control)
        self.coxa = sim.getObject(f'/leg_{leg_number}_joint_1')
        self.femur = sim.getObject(f'/leg_{leg_number}_joint_2')
        self.tibia = sim.getObject(f'/leg_{leg_number}_joint_3')
        
        # Store initial foot position (home position)
        self.initial_pos = sim.getObjectPosition(self.foot_tip, self.leg_base)
        print(f"Leg {leg_number} initial position: {self.initial_pos}")
        
        # Setup IK
        self._setup_ik()
        
        # Gait parameters
        self.step_progression = 0.0
        self.real_movement_strength = 0.0
        
        # Movement data
        self.mov_data = {
            'vel': 0.5,
            'amplitude': 0.16,
            'height': 0.04,
            'dir': 0.0,
            'rot': 0.0,
            'strength': 0.0
        }
        
        print(f"Initialized leg {leg_number} at {math.degrees(mounting_angle):.1f} degrees")
    
    def _setup_ik(self):
        """Setup IK environment and group"""
        try:
            self.ik_env = self.simIK.createEnvironment()
            self.ik_group = self.simIK.createGroup(self.ik_env)
            
            self.simIK.addElementFromScene(
                self.ik_env,
                self.ik_group,
                self.base,
                self.foot_tip,
                self.foot_target,
                self.simIK.constraint_position
            )
            
            print(f"  IK setup successful")
        except Exception as e:
            print(f"  IK setup failed: {e}")
    
    def set_movement_params(self, velocity=0.5, amplitude=0.16, height=0.04, 
                           direction=0.0, rotation=0.0, strength=1.0):
        """
        Set movement parameters
        
        Args:
            velocity: Step speed (higher = faster steps)
            amplitude: Step size in meters
            height: Lift height in meters
            direction: Movement direction in degrees (0=forward, 90=right, etc.)
            rotation: Rotation mode (0=no rotation, 1=rotate clockwise, -1=counter)
            strength: Movement strength (0=stopped, 1=full speed)
        """
        self.mov_data['vel'] = velocity
        self.mov_data['amplitude'] = amplitude
        self.mov_data['height'] = height
        self.mov_data['dir'] = math.radians(direction)
        self.mov_data['rot'] = rotation
        self.mov_data['strength'] = strength
    
    def calculate_foot_offset(self, step_phase):
        """
        Calculate foot offset for current step phase (like Lua code)
        
        Args:
            step_phase: Current phase in gait cycle (0 to 1)
            
        Returns:
            [x_offset, y_offset, z_offset] in leg's local frame
        """
        sp = step_phase % 1.0
        offset = [0.0, 0.0, 0.0]
        
        # 4-phase gait cycle
        if sp < (1/3):
            # Phase 1: Stance - foot on ground moving backward
            offset[0] = sp * 3 * self.mov_data['amplitude'] / 2
            
        elif sp < (1/3 + 1/6):
            # Phase 2: Lift - foot lifts while moving forward
            s = sp - 1/3
            offset[0] = self.mov_data['amplitude']/2 - self.mov_data['amplitude']*s*6/2
            offset[2] = s * 6 * self.mov_data['height']
            
        elif sp < (2/3):
            # Phase 3: Swing - foot moves forward at height
            s = sp - 1/3 - 1/6
            offset[0] = -self.mov_data['amplitude'] * s * 6 / 2
            offset[2] = (1 - s*6) * self.mov_data['height']
            
        else:
            # Phase 4: Lower - foot returns to ground
            s = sp - 2/3
            offset[0] = -self.mov_data['amplitude'] * (1 - s*3) / 2
        
        return offset
    
    def transform_offset_to_world(self, offset):
        """
        Transform offset from leg frame to world frame
        FIXED: Corrected coordinate system so forward=forward, strafe=strafe
        
        Args:
            offset: [x, y, z] offset in leg's local frame
            
        Returns:
            [x, y, z] offset in world frame
        """
        # Calculate movement direction for this leg
        md = self.mov_data['dir'] + abs(self.mov_data['rot']) * math.atan2(
            self.initial_pos[0] * self.mov_data['rot'],
            -self.initial_pos[1] * self.mov_data['rot']
        )
        
        # FIXED: Swap sin and cos to correct the coordinate system
        # This makes 0 deg = forward, 90 deg = right
        offset_world = [
            offset[0] * math.sin(md) * self.real_movement_strength,   # SWAPPED (was cos)
            offset[0] * math.cos(md) * self.real_movement_strength,   # SWAPPED (was sin)
            offset[2] * self.real_movement_strength
        ]
        
        return offset_world
    
    def update(self, dt):
        """
        Update leg position (call this every simulation step)
        
        Args:
            dt: Time step (seconds)
        """
        # Smooth ramping of movement strength
        dx = self.mov_data['strength'] - self.real_movement_strength
        if abs(dx) > dt * 0.1:
            dx = abs(dx) * dt * 0.5 / dx
        self.real_movement_strength = self.real_movement_strength + dx
        
        # Calculate foot offset for current phase
        offset_local = self.calculate_foot_offset(self.step_progression)
        
        # Transform to world coordinates
        offset_world = self.transform_offset_to_world(offset_local)
        
        # Calculate target foot position
        target_pos = [
            self.initial_pos[0] + offset_world[0],
            self.initial_pos[1] + offset_world[1],
            self.initial_pos[2] + offset_world[2]
        ]
        
        # Set foot target position (IK will calculate joint angles automatically!)
        self.sim.setObjectPosition(self.foot_target, target_pos, self.leg_base)
        
        # Solve IK
        self.simIK.handleGroup(self.ik_env, self.ik_group, {'syncWorlds': True, 'allowError': True})
        
        # Update step progression
        self.step_progression = self.step_progression + dt * self.mov_data['vel']
    
    def get_joint_angles(self):
        """Get current joint angles (for monitoring or sending to real robot)"""
        return {
            'coxa': self.sim.getJointPosition(self.coxa),
            'femur': self.sim.getJointPosition(self.femur),
            'tibia': self.sim.getJointPosition(self.tibia)
        }


# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    print("="*70)
    print("HEXAPOD IK-BASED CONTROLLER")
    print("="*70)
    
    # Connect to CoppeliaSim
    print("\nConnecting to CoppeliaSim...")
    client = RemoteAPIClient(port=23000)
    sim = client.getObject('sim')
    
    # Load simIK module
    try:
        simIK = client.require('simIK')
        print("Loaded simIK module")
    except:
        print("ERROR: Could not load simIK module")
        return
    
    print("Connected!")
    
    # Create leg controller
    mounting_angle = math.radians(33)
    leg = HexapodLegIK(sim, simIK, leg_number=1, mounting_angle=mounting_angle)
    
    # Start simulation
    sim.startSimulation()
    print("\nSimulation started")
    time.sleep(1)
    
    # Test 1: Walk forward
    print("\n" + "="*70)
    print("TEST 1: WALKING FORWARD")
    print("="*70)
    leg.set_movement_params(
        velocity=0.9,
        amplitude=0.11,
        height=0.02,
        direction=0,         # 0 = forward
        rotation=0,
        strength=1.0
    )
    
    start_time = time.time()
    duration = 10.0
    
    while (time.time() - start_time) < duration:
        dt = sim.getSimulationTimeStep()
        leg.update(dt)
        time.sleep(dt)
    
    # Test 2: Stop
    print("\n" + "="*70)
    print("TEST 2: STOPPING")
    print("="*70)
    leg.set_movement_params(strength=0.0)
    
    start_time = time.time()
    while (time.time() - start_time) < 3.0:
        dt = sim.getSimulationTimeStep()
        leg.update(dt)
        time.sleep(dt)
    
    # Test 3: Strafe right
    print("\n" + "="*70)
    print("TEST 3: STRAFING RIGHT")
    print("="*70)
    leg.set_movement_params(
        direction=90,        # 90 = right
        strength=1.0
    )
    
    start_time = time.time()
    while (time.time() - start_time) < 8.0:
        dt = sim.getSimulationTimeStep()
        leg.update(dt)
        time.sleep(dt)
    
    # Test 4: Rotate
    print("\n" + "="*70)
    print("TEST 4: ROTATING")
    print("="*70)
    leg.set_movement_params(
        direction=0,
        rotation=1.0,
        strength=1.0
    )
    
    start_time = time.time()
    while (time.time() - start_time) < 8.0:
        dt = sim.getSimulationTimeStep()
        leg.update(dt)
        time.sleep(dt)
    
    # Stop
    print("\n" + "="*70)
    print("STOPPING AND ENDING")
    print("="*70)
    leg.set_movement_params(strength=0.0)
    
    start_time = time.time()
    while (time.time() - start_time) < 2.0:
        dt = sim.getSimulationTimeStep()
        leg.update(dt)
        time.sleep(dt)
    
    sim.stopSimulation()
    print("\nDone!")

if __name__ == "__main__":
    main()