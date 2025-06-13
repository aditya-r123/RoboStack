import numpy as np
from controllers.pid import PID
"""
Where we'll put higher-level logic: choosing which block to pick,
what target pose to place it at, colour detection, etc.
For now, only a dummy policy that does nothing.
"""

class IdlePolicy:
    def reset(self, env):
        pass

    def step(self, obs):
        # Return zeros -> the PID above will hold current joint positions.
        return env.robots[0].controller.control(np.zeros_like(
            env.robots[0].controller.control_limits[0]
        ))


class StackPolicy(object):
    """
    States
    0 : Hover above cubeA, gripper open
    1 : Descend to cubeA centre, gripper open
    2 : close gripper and lift cubeA to a safe height
    3 : translate to point above cubeB
    4 : Descend until cubeA is just over cubeB
    5 : Open gripper to release
    """

    def __init__(self, obs):
        self.hover_h   = 0.06         
        self.lift_h    = 0.10          
        self.stack_gap = 0.02          

        self.state = 0

        # first target: hover over cubeA
        self.cur_target = obs["cubeA_pos"].copy()
        self.cur_target[2] += self.hover_h

        self.pid = PID(1.4, 1, 0.0, self.cur_target)

    def reached(self, ee_pos, tol=1e-4):
        return np.sum((ee_pos - self.cur_target) ** 2) < tol

    def get_action(self, obs):
        ee_pos = obs["robot0_eef_pos"]
        cubeA  = obs["cubeA_pos"]
        cubeB  = obs["cubeB_pos"]

        action = np.zeros(7)              



        # Move grappler abobe cubeA and keep it open
        if self.state == 0:                     
            action[-1] = -1                      
            if self.reached(ee_pos):
                self._set_state(1)
                self.cur_target = cubeA.copy()
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action

        # descend grappler to cubeA
        if self.state == 1:                     
            action[-1] = -1
            if self.reached(ee_pos):
                self._set_state(2)
                self.cur_target = cubeA.copy()
                self.cur_target[2] += self.lift_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action

        #grasp cubeA and life it
        if self.state == 2:                      
            action[-1] = 1                     
            if self.reached(ee_pos):
                self._set_state(3)
                self.cur_target = cubeB.copy()
                self.cur_target[2] += self.lift_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action


        # move cubeA above cubeB
        if self.state == 3:                     
            action[-1] = 1
            if self.reached(ee_pos):
                self._set_state(4)
                self.cur_target = cubeB.copy()
                self.cur_target[2] += self.stack_gap
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action


        # lower cubeA onto cubeB
        if self.state == 4:                     
            action[-1] = 1
            if self.reached(ee_pos):
                self._set_state(5)
                self.cur_target = ee_pos.copy()
                self.cur_target[2] += self.hover_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action

        # release cubeA to fall onto cubeB
        action[-1] = -1
        if not self.reached(ee_pos):
            action[:3] = self.pid.update(ee_pos)
        return action


class MultiColorStackPolicy(object):
    def __init__(self, obs):
        self.hover_h   = 0.05         
        self.lift_h    = 0.07          
        self.descend_h =  - 0.025
        self.stack_gap = 0.005
        self.block_height = obs['block_size'][2]
        self.wait_steps = 10

        # Recovery parameters
        self.max_stuck_steps = 250
        self.max_retries = 3
        self.rotation_duration = 10
        self.stuck_counter = 0
        self.retry_counter = 0
        self.rotation_counter = 0
        self.pre_recovery_state = 0
        self.pre_recovery_target = None

        self._build_plan(obs)
        self.current_task_idx = 0
        if not self.plan:
            self.done = True
        else:
            print(f"Starting MultiColorStackPolicy with {len(self.plan)} tasks.")
            self._setup_for_current_task(obs)

    def _build_plan(self, obs):
        self.plan = []
        block_positions = obs['block_positions']
        block_colors = obs['block_colors']
        
        colors = np.unique(block_colors)
        
        self.stack_tops = {}

        for color in colors:
            color_indices = np.where(block_colors == color)[0]
            if len(color_indices) > 1:
                base_block_idx = color_indices[0]
                base_block_pos = block_positions[base_block_idx]
                self.stack_tops[color] = {'pos': base_block_pos, 'height': 1}

                for i in range(1, len(color_indices)):
                    block_to_stack_idx = color_indices[i]
                    self.plan.append({
                        'source_idx': block_to_stack_idx,
                        'target_color': color
                    })
        print(f"Built plan with {len(self.plan)} tasks.")

    def _set_state(self, new_state):
        if self.state != new_state:
            print(f"State transition: {self.state} -> {new_state}")
            self.state = new_state
            self.stuck_counter = 0

    def _setup_for_current_task(self, obs):
        if self.current_task_idx >= len(self.plan):
            self.done = True
            return

        self.retry_counter = 0

        self.done = False
        task = self.plan[self.current_task_idx]
        source_idx = task['source_idx']
        target_color = task['target_color']
        
        self.source_pos = obs['block_positions'][source_idx]
        
        target_stack_info = self.stack_tops[target_color]
        self.target_pos = target_stack_info['pos'].copy()
        self.target_pos[2] += target_stack_info['height'] * self.block_height

        self.state = 0
        
        self.cur_target = self.source_pos.copy()
        self.cur_target[2] += self.hover_h
        
        self.pid = PID(1.4, 1, 0.0, self.cur_target)

    def reached(self, ee_pos, tol=1e-3):
        return np.sum((ee_pos - self.cur_target) ** 2) < tol

    def get_action(self, obs):
        if self.done:
            print("MultiColorStackPolicy: Done.")
            return np.zeros(7)

        ee_pos = obs["robot0_eef_pos"]
        
        current_task = self.plan[self.current_task_idx]
        source_idx = current_task['source_idx']
        
        action = np.zeros(7)

        # Stuck detection and recovery logic
        if self.state < 9:  # Not in a recovery state
            self.stuck_counter += 1
            if self.stuck_counter > self.max_stuck_steps:
                print(f"Stuck in state {self.state}! Attempting recovery...")
                self.retry_counter += 1
                if self.retry_counter > self.max_retries:
                    print(f"Max retries ({self.max_retries}) exceeded for task {self.current_task_idx}. Skipping task.")
                    self.current_task_idx += 1
                    if self.current_task_idx >= len(self.plan):
                        self.done = True
                    else:
                        self._setup_for_current_task(obs)
                    return np.zeros(7)
                
                self.pre_recovery_state = self.state
                self.pre_recovery_target = self.cur_target.copy()
                self._set_state(9)  # Start recovery: Lift
                self.cur_target = ee_pos.copy()
                self.cur_target[2] += self.lift_h
                self.pid.reset(self.cur_target)

        if self.state == 0: # Hover above source cube, gripper open
            print(f"State 0: Hover above source cube {source_idx}")
            action[-1] = -1
            if self.reached(ee_pos):
                self._set_state(1)
                self.cur_target = obs['block_positions'][source_idx].copy()
                self.cur_target[2] += self.descend_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action

        if self.state == 1: # Descend to source cube, gripper open
            print(f"State 1: Descend to source cube {source_idx}")
            action[-1] = -1
            if self.reached(ee_pos):
                self._set_state(2)
                self.cur_target = obs['block_positions'][source_idx].copy()
                self.cur_target[2] += self.lift_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action

        if self.state == 2: # Grasp source cube and lift it
            print(f"State 2: Grasp source cube {source_idx} and lift it")
            action[-1] = 1
            if self.reached(ee_pos):
                self._set_state(3)
                self.cur_target = self.target_pos.copy()
                self.cur_target[2] += self.lift_h
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action
        
        if self.state == 3: # Move source cube above target stack
            print("State 3: Move source cube above target stack")
            action[-1] = 1
            if self.reached(ee_pos):
                self._set_state(4)
                self.cur_target = self.target_pos.copy()
                self.cur_target[2] += self.stack_gap
                self.pid.reset(self.cur_target)
            action[:3] = self.pid.update(ee_pos)
            return action
            
        if self.state == 4: # Lower source cube onto target stack
            print("State 4: Lower source cube onto target stack")
            action[-1] = 1
            if self.reached(ee_pos):
                self._set_state(5)
                self.wait_counter = 0
            action[:3] = self.pid.update(ee_pos)
            return action

        if self.state == 5: # Release cube and wait
            print(f"State 5: Release cube and wait ({self.wait_counter}/{self.wait_steps})")
            action[-1] = -1 # open gripper
            self.wait_counter += 1
            if self.wait_counter > self.wait_steps:
                self._set_state(6)
                # Setup for moving up
                self.cur_target = ee_pos.copy()
                self.cur_target[2] += self.hover_h
                self.pid.reset(self.cur_target)
            
            # Hold position while waiting
            action[:3] = [0,0,0]
            return action

        if self.state == 6: # Move up after release
            print("State 6: Move up after release")
            action[-1] = -1
            
            if self.reached(ee_pos, tol=1e-2):
                task = self.plan[self.current_task_idx]
                target_color = task['target_color']
                
                new_top_pos = self.target_pos.copy()
                self.stack_tops[target_color]['pos'] = new_top_pos
                self.stack_tops[target_color]['height'] += 1

                self.current_task_idx += 1
                if self.current_task_idx >= len(self.plan):
                    self.done = True
                    return np.zeros(7)

                print(f"Task {self.current_task_idx}/{len(self.plan)} complete. Starting next task.")
                self._set_state(7) # Go to safe Z
                
                max_stack_z = 0.0
                for color_info in self.stack_tops.values():
                    stack_top_z = color_info['pos'][2] + (color_info['height'] - 0.5) * self.block_height
                    if stack_top_z > max_stack_z:
                        max_stack_z = stack_top_z
                
                self.safe_z = max_stack_z + self.lift_h + self.hover_h

                self.cur_target = ee_pos.copy()
                self.cur_target[2] = self.safe_z
                self.pid.reset(self.cur_target)
                
            action[:3] = self.pid.update(ee_pos)
            return action

        if self.state == 7: # Move to safe Z height
            print("State 7: Move to safe Z height")
            action[-1] = -1 # Keep gripper open
            if self.reached(ee_pos):
                self._set_state(8)
                
                next_task = self.plan[self.current_task_idx]
                next_source_idx = next_task['source_idx']
                next_source_pos = obs['block_positions'][next_source_idx]

                self.cur_target = next_source_pos.copy()
                self.cur_target[2] = self.safe_z
                self.pid.reset(self.cur_target)
            
            action[:3] = self.pid.update(ee_pos)
            return action

        if self.state == 8: # Move above next source block at safe Z
            print("State 8: Move above next source block at safe Z")
            action[-1] = -1
            if self.reached(ee_pos):
                self._setup_for_current_task(obs)

            action[:3] = self.pid.update(ee_pos)
            return action
        
        # --- Recovery States ---
        if self.state == 9:  # 9: Recovery Lift
            print(f"Recovery State 9: Lifting up. Retry {self.retry_counter}/{self.max_retries}.")
            action[:3] = self.pid.update(ee_pos)
            if self.reached(ee_pos):
                self._set_state(10)
                self.rotation_counter = 0
            return action

        if self.state == 10:  # 10: Recovery Rotate
            print(f"Recovery State 10: Rotating ({self.rotation_counter}/{self.rotation_duration}).")
            action[5] = 0.5  # Rotate yaw
            self.rotation_counter += 1
            if self.rotation_counter > self.rotation_duration:
                self._set_state(self.pre_recovery_state)
                self.cur_target = self.pre_recovery_target
                self.pid.reset(self.cur_target)
            return action

        return np.zeros(7)

