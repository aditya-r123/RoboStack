
class MultiStackPolicy(object):
    """
    States for each block:
    0: Hover above block, gripper open
    1: Descend to block center, gripper open
    2: Close gripper and lift block
    3: Translate to point above target stack
    4: Descend until block is just above stack
    5: Open gripper to release
    """

    def __init__(self, obs):
        # PID controller for position control
        self.pid_controller = PID(
            kp=[2.0, 2.0, 3.0],  # Increased z-axis gain from 2.0 to 3.0
            ki=[0.1, 0.1, 0.2],  # Increased z-axis gain from 0.1 to 0.2
            kd=[0.5, 0.5, 0.8],  # Increased z-axis gain from 0.5 to 0.8
            target=np.zeros(3)  # Initial target will be updated in get_action
        )
        
        # Height parameters
        self.hover_h = 0.06
        self.lift_h = 0.2
        self.grasp_h = -0.02
        self.stack_gap = 0.02
        self.small_lift = 0.05
        self.safe_lift = 0.15
        self.gripper_close_threshold = 0.02
        
        # Initialize state machine
        self.state = 0
        self.current_block_idx = 0
        self.stacked_blocks = set()
        self.color_stacks = {}  # Maps color to (position, height)
        self.block_positions = obs['block_positions']
        self.block_colors = obs['block_colors']
        self.n_blocks = len(self.block_positions)
        self.color_base_blocks = {}  # Maps color to index of first block of that color
        self.first_block_positions = {}  # Maps color to actual position of first block placed
        self.base_blocks = set()  # Set of block indices that are bases
        self.base_positions = {}  # Maps color to actual position of base block after placement
        
        # Add rotation recovery variables
        self.rotation_timer = 0
        self.rotation_time = 10  # Number of steps to rotate
        self.rotation_angle = 0.5  # Amount to rotate (in radians)
        self.grasp_attempts = 0
        self.max_grasp_attempts = 3  # Maximum number of grasp attempts before moving to next block
        
        # Add state timers for stuck detection
        self.state_timer = 0
        self.max_state_time = 300  # Maximum time to stay in a state before recovery
        
        # Add reset state variables
        self.reset_timer = 0
        self.reset_time = 50  # Number of steps to stay in reset state
        self.neutral_position = np.array([0.0, 0.0, 1.25])  # Neutral position above table center
        self.neutral_joints = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.8])  # Neutral joint configuration
        self.joint_reset_time = 20  # Number of steps to stay in joint reset
        
        # Set initial target to hover above first block
        self.cur_target = self.block_positions[0].copy()
        self.cur_target[2] += self.hover_h
        
        # Initialize PID controller for position
        self.pos_pid = PID(1.4, 1, 0.0, self.cur_target)
        
        # Add timer for grasping
        self.grasp_timer = 0
        self.grasp_wait_time = 20  # Number of steps to wait while grasping

    def get_block_color(self, block_pos):
        """
        Get the color of a block based on its position.
        Uses the block_colors array from observations.
        """
        # Find the index of the block with this position
        block_idx = np.where(np.all(self.block_positions == block_pos, axis=1))[0][0]
        # Return the color from our stored colors array
        return self.block_colors[block_idx]

    def get_stack_position(self, color):
        if color not in self.color_stacks:
            # For first block of this color, use its actual position as base
            first_block_pos = self.block_positions[self.current_block_idx].copy()
            self.first_block_positions[color] = first_block_pos
            self.color_stacks[color] = (first_block_pos, 0)
            self.base_blocks.add(self.current_block_idx)  # Mark this block as a base
            print(f"New base block for color {color} at position {first_block_pos}")
            return first_block_pos, 0
        else:
            # For subsequent blocks, use the actual base position
            if color in self.base_positions:
                base_pos = self.base_positions[color]
            else:
                base_pos = self.first_block_positions[color]
            _, height = self.color_stacks[color]
            print(f"Stack position for color {color}: base={base_pos}, height={height}")
            return base_pos, height

    def reached(self, ee_pos, tol=5e-4):
        return np.sum((ee_pos - self.cur_target) ** 2) < tol

    def get_action(self, obs):
        # Update PID target based on current state
        if self.state == 0:  # Move to block
            target_pos = self.cur_target.copy()
            target_pos[2] += self.hover_h
            self.pid_controller.reset(target=target_pos)
        elif self.state == 1:  # Descend to grasp
            target_pos = self.cur_target.copy()
            target_pos[2] += self.grasp_h
            self.pid_controller.reset(target=target_pos)
        elif self.state == 2:  # Lift block
            target_pos = self.cur_target.copy()
            target_pos[2] += self.lift_h
            self.pid_controller.reset(target=target_pos)
        elif self.state == 3:  # Move to stack
            target_pos = self.cur_target.copy()
            target_pos[2] += self.safe_lift
            self.pid_controller.reset(target=target_pos)
        elif self.state == 4:  # Descend to stack
            target_pos = self.cur_target.copy()
            target_pos[2] += self.stack_gap
            self.pid_controller.reset(target=target_pos)
        elif self.state == 5:  # Release
            target_pos = self.cur_target.copy()
            self.pid_controller.reset(target=target_pos)
        elif self.state == 2.5:  # Small lift
            target_pos = self.cur_target.copy()
            target_pos[2] += self.small_lift
            self.pid_controller.reset(target=target_pos)
        elif self.state == 2.7:  # Reset position
            target_pos = self.neutral_position.copy()
            self.pid_controller.reset(target=target_pos)
        elif self.state == 2.6:  # Rotation recovery
            target_pos = self.cur_target.copy()
            target_pos[2] += self.hover_h
            self.pid_controller.reset(target=target_pos)

        # Get current end effector position
        current_pos = obs['robot0_eef_pos']
        
        # Compute PID control signal
        action = np.zeros(7)  # 3 for position, 3 for orientation, 1 for gripper
        action[:3] = self.pid_controller.update(current_pos)

        # If we've stacked all blocks, return zero action
        if len(self.stacked_blocks) == self.n_blocks:
            return action

        # Get current block and its color
        current_block_pos = self.block_positions[self.current_block_idx]
        block_color = self.get_block_color(current_block_pos)
        stack_pos, stack_height = self.get_stack_position(block_color)
        print(f"Current block {self.current_block_idx} of color {block_color}")
        print(f"Current position: {current_block_pos}")
        print(f"Target stack position: {stack_pos} at height {stack_height}")
        print(f"End effector position: {current_pos}")

        # Helper function to check if we're stuck and need recovery
        def check_stuck():
            if self.state_timer > self.max_state_time:
                print(f"Stuck in state {self.state} for too long, attempting reset...")
                self.state = 2.7  # Move to reset state
                self.cur_target = self.neutral_position.copy()
                self.pos_pid.reset(self.cur_target)
                self.reset_timer = 0
                self.state_timer = 0
                return True
            return False

        # State machine for stacking process
        if self.state == 2.7:  # Reset state
            action[-1] = -1  # Open gripper
            
            # First reset joint angles
            if self.reset_timer < self.joint_reset_time:
                # Set joint angles to neutral configuration
                action[3:6] = self.neutral_joints[:3]  # First 3 joint angles
                action[6] = self.neutral_joints[3]  # Last joint angle
                self.reset_timer += 1
                print("State 2.7 action (joint reset)", action[3:], "Timer:", self.reset_timer)
            # Then move to neutral position
            elif self.reset_timer < self.reset_time:
                if self.reached(current_pos):
                    # After reaching neutral position, try rotation recovery
                    print("Reset complete, attempting rotation recovery...")
                    self.state = 2.6  # Move to rotation recovery state
                    self.cur_target = current_block_pos.copy()
                    self.cur_target[2] += self.hover_h
                    self.pos_pid.reset(self.cur_target)
                    self.rotation_timer = 0
                    self.grasp_attempts += 1
                else:
                    self.reset_timer += 1
                    pos_action = self.pos_pid.update(current_pos)
                    action[:3] = pos_action
                    print("State 2.7 action (position reset)", action[:3], "Timer:", self.reset_timer)
            else:
                # If we've been in reset too long, force move to rotation recovery
                print("Reset timeout, forcing rotation recovery...")
                self.state = 2.6
                self.cur_target = current_block_pos.copy()
                self.cur_target[2] += self.lift_h  # Use lift_h instead of hover_h for higher position
                self.pos_pid.reset(self.cur_target)
                self.rotation_timer = 0
                self.grasp_attempts += 1
            
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 2.6:  # Rotation recovery state
            action[-1] = -1  # Open gripper
            if self.reached(current_pos):  # Reached hover height
                if self.rotation_timer < self.rotation_time:
                    # Rotate around z-axis
                    action[3:6] = [0, 0, self.rotation_angle]
                    self.rotation_timer += 1
                    print("State 2.6 action (rotating)", action[3:6], "Timer:", self.rotation_timer)
                else:
                    # Done rotating, try grasping again
                    self.state = 1
                    self.cur_target = current_block_pos.copy()
                    self.cur_target[2] += self.grasp_h
                    self.pos_pid.reset(self.cur_target)
            else:
                pos_action = self.pos_pid.update(current_pos)
                action[:3] = pos_action
            return action

        elif self.state == 0:  # Hover above block
            action[-1] = -1  # Open gripper
            if self.reached(current_pos):
                # If this is a base block, we don't need to lift it
                if self.current_block_idx in self.base_blocks:
                    self.state = 5  # Skip to release state for base blocks
                    self.cur_target = current_pos.copy()
                    self.cur_target[2] += self.hover_h
                    self.pos_pid.reset(self.cur_target)
                else:
                    self.state = 1
                    self.cur_target = current_block_pos.copy()
                    self.cur_target[2] += self.grasp_h
                    self.pos_pid.reset(self.cur_target)
                self.state_timer = 0  # Reset timer when changing states
            else:
                self.state_timer += 1
                if check_stuck():
                    return action
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print("State 0 action", action[:3])
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 1:  # Descend to block
            # Calculate how close we are to the target height
            height_diff = current_pos[2] - self.cur_target[2]
            
            # Binary gripper control - either fully open or fully closed
            if height_diff < self.gripper_close_threshold:
                action[-1] = 1  # Fully close gripper when close to block
            else:
                action[-1] = -1  # Keep gripper fully open when far from block
            
            if self.reached(current_pos):
                self.state = 2
                self.grasp_timer = 0  # Reset grasp timer
                self.state_timer = 0  # Reset timer when changing states
            else:
                self.state_timer += 1
                if check_stuck():
                    return action
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print("State 1 action", action[:3], "Gripper:", action[-1])
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 2:  # Grasp and wait
            action[-1] = 1  # Close gripper
            if self.grasp_timer < self.grasp_wait_time:
                self.grasp_timer += 1
                # Stay at current position while grasping
                pos_action = self.pos_pid.update(current_pos)
                action[:3] = pos_action
                print("State 2 action (grasping)", action[:3])
            else:
                # After waiting, move to small lift state
                self.state = 2.5
                self.cur_target = current_block_pos.copy()
                self.cur_target[2] += self.small_lift  # Lift slightly
                self.pos_pid.reset(self.cur_target)
                self.state_timer = 0  # Reset timer when changing states
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 2.5:  # Small lift to verify grasp
            action[-1] = 1  # Keep gripper closed
            # Calculate height difference regardless of whether we've reached target
            height_diff = current_pos[2] - current_block_pos[2]
            
            if self.reached(current_pos):
                # Check if we successfully lifted the block by comparing current height to original block height
                # Allow for some tolerance in the height difference
                if height_diff > 0.02:  # Block has been lifted at least 2cm from original position
                    # If we successfully lifted, move to safe height before going to stack
                    self.state = 2.8  # New state for safe lift
                    self.cur_target = current_block_pos.copy()
                    self.cur_target[2] += self.safe_lift  # Lift to safe height
                    self.pos_pid.reset(self.cur_target)
                    self.grasp_attempts = 0  # Reset grasp attempts counter
                    self.state_timer = 0  # Reset timer when changing states
                else:  # Block wasn't lifted, try recovery
                    if self.grasp_attempts < self.max_grasp_attempts:
                        print("Grasp failed, attempting recovery...")
                        self.state = 2.6  # Move to rotation recovery state
                        self.cur_target = current_block_pos.copy()
                        self.cur_target[2] += self.hover_h  # Move back to hover height
                        self.pos_pid.reset(self.cur_target)
                        self.rotation_timer = 0
                        self.grasp_attempts += 1
                        self.state_timer = 0  # Reset timer when changing states
                    else:
                        print("Max grasp attempts reached, moving to next block")
                        self.stacked_blocks.add(self.current_block_idx)
                        self.current_block_idx += 1
                        if self.current_block_idx >= self.n_blocks:
                            self.current_block_idx = 0
                        self.state = 0
                        self.cur_target = self.block_positions[self.current_block_idx].copy()
                        self.cur_target[2] += self.hover_h
                        self.pos_pid.reset(self.cur_target)
                        self.grasp_attempts = 0
                        self.state_timer = 0  # Reset timer when changing states
            else:
                self.state_timer += 1
                if check_stuck():
                    return action
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print("State 2.5 action (small lift)", action[:3], "Height diff:", height_diff)
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 2.8:  # Safe lift before lateral movement
            action[-1] = 1  # Keep gripper closed
            if self.reached(current_pos):
                # After reaching safe height, move to stack position
                self.state = 3
                self.cur_target = stack_pos.copy()
                self.cur_target[2] += stack_height + self.stack_gap
                self.pos_pid.reset(self.cur_target)
                self.state_timer = 0  # Reset timer when changing states
            else:
                self.state_timer += 1
                if check_stuck():
                    return action
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print("State 2.8 action (safe lift)", action[:3])
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 3:  # Move above stack
            action[-1] = 1  # Keep gripper closed
            if self.reached(current_pos):
                self.state = 4
                self.cur_target = stack_pos.copy()
                self.cur_target[2] += stack_height + self.stack_gap
                self.pos_pid.reset(self.cur_target)
                self.state_timer = 0  # Reset timer when changing states
                print(f"Reached stack position, moving to height {self.cur_target[2]}")
            else:
                self.state_timer += 1
                if self.state_timer > self.max_state_time:
                    print("Stuck while moving to stack, attempting joint reset...")
                    # Store current target for after reset
                    self.saved_target = self.cur_target.copy()
                    # Move to a slightly higher position to avoid collisions
                    self.cur_target = current_pos.copy()
                    self.cur_target[2] += 0.1  # Move up 10cm
                    self.pos_pid.reset(self.cur_target)
                    self.state = 3.5  # Move to joint reset state
                    self.state_timer = 0
                    return action
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print(f"State 3 action: {action[:3]}, Current height: {current_pos[2]}, Target height: {self.cur_target[2]}")
            action[3:6] = [0, 0, 0]  # Look down
            return action

        elif self.state == 3.5:  # Joint reset while holding block
            action[-1] = 1  # Keep gripper closed
            if self.reached(current_pos):
                # Reset joint angles while maintaining height
                action[3:6] = self.neutral_joints[:3]  # First 3 joint angles
                action[6] = self.neutral_joints[3]  # Last joint angle
                self.state_timer += 1
                if self.state_timer >= self.joint_reset_time:
                    # Return to original target
                    self.state = 3
                    self.cur_target = self.saved_target.copy()
                    self.pos_pid.reset(self.cur_target)
                    self.state_timer = 0
                    print(f"Joint reset complete, returning to stack position: {self.cur_target}")
            else:
                pos_action = self.pos_pid.update(current_pos)
                action[:3] = pos_action
            print(f"State 3.5 action (joint reset while holding): {action[3:]}, Timer: {self.state_timer}")
            return action

        elif self.state == 4:  # Lower onto stack
            action[-1] = 1
            if self.reached(current_pos):
                self.state = 5
                self.cur_target = current_pos.copy()
                self.cur_target[2] += self.hover_h
                self.pos_pid.reset(self.cur_target)
                print(f"Reached stack height, moving to hover position: {self.cur_target}")
            pos_action = self.pos_pid.update(current_pos)
            action[:3] = pos_action
            print(f"State 4 action: {action[:3]}, Current height: {current_pos[2]}, Target height: {self.cur_target[2]}")
            action[3:6] = [0, 0, 0]  # Look down
            return action

        else:  # Release and move to next block
            action[-1] = -1  # Open gripper
            if not self.reached(current_pos):
                pos_action = self.pos_pid.update(current_pos)
                action[:3] = pos_action
            else:
                # Update stack height and move to next block
                if self.current_block_idx in self.base_blocks:
                    # Store the actual position of the base block
                    self.base_positions[block_color] = current_pos.copy()
                elif self.current_block_idx not in self.base_blocks:  # Only update height for non-base blocks
                    self.color_stacks[block_color] = (stack_pos, stack_height + self.stack_gap)
                self.stacked_blocks.add(self.current_block_idx)
                
                # Find next unstacked block that is not a base
                while (self.current_block_idx + 1) < self.n_blocks:
                    self.current_block_idx += 1
                    if self.current_block_idx not in self.stacked_blocks and self.current_block_idx not in self.base_blocks:
                        break
                
                # If we've gone through all non-base blocks, start over with remaining blocks
                if self.current_block_idx >= self.n_blocks:
                    self.current_block_idx = 0
                    while self.current_block_idx < self.n_blocks:
                        if self.current_block_idx not in self.stacked_blocks and self.current_block_idx not in self.base_blocks:
                            break
                        self.current_block_idx += 1
                
                # Reset state for next block
                self.state = 0
                self.cur_target = self.block_positions[self.current_block_idx].copy()
                self.cur_target[2] += self.hover_h
                self.pos_pid.reset(self.cur_target)
            
            action[3:6] = [0, 0, 0]  # Look down
            return action

