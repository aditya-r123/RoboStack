# # # from collections import OrderedDict
# # # import numpy as np
# # # from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
# # # from robosuite.models.objects import BoxObject
# # # from robosuite.models.arenas import TableArena 
# # # from robosuite.utils.placement_samplers import UniformRandomSampler
# # # from robosuite.utils.observables import Observable, sensor
# # # import robosuite.utils.transform_utils as T 

# # # class CustomMultiStackEnv(ManipulationEnv):
# # #     """
# # #     A custom Robosuite environment for stacking multiple red blocks and multiple green blocks.
# # #     Uses TableArena for the workspace.
# # #     """
# # #     def __init__(
# # #         self,
# # #         robots, 
# # #         controller_configs=None, 
# # #         gripper_types="default", 
# # #         initialization_noise=None, 
# # #         table_full_size=(0.8, 0.8, 0.05), 
# # #         use_camera_obs=False, 
# # #         # reward_scale=1.0, # This parameter is removed
# # #         reward_shaping=False, # Keep reward_shaping if it's still supported
# # #         has_renderer=False, 
# # #         has_offscreen_renderer=True, 
# # #         render_camera="frontview", 
# # #         render_collision_mesh=False, 
# # #         render_visual_mesh=True, 
# # #         render_gpu_device_id=-1, 
# # #         control_freq=20, 
# # #         horizon=1000, 
# # #         ignore_done=False, 
# # #         hard_reset=True, 
# # #         camera_names="agentview", 
# # #         camera_heights=256, 
# # #         camera_widths=256, 
# # #         camera_depths=False, 
# # #         num_red_blocks=2, 
# # #         num_green_blocks=2, 
# # #         block_size=0.022, 
# # #     ):
# # #         self.num_red_blocks = num_red_blocks
# # #         self.num_green_blocks = num_green_blocks
# # #         self.block_half_size = block_size 

# # #         self.red_block_names = [f"red_cube_{i}" for i in range(num_red_blocks)]
# # #         self.green_block_names = [f"green_cube_{i}" for i in range(num_green_blocks)]
# # #         self.all_block_names = self.red_block_names + self.green_block_names

# # #         self.placement_initializer = None
        
# # #         self.table_offset = np.array((0, 0, 0.8)) 
# # #         self.table_full_size = table_full_size
        
# # #         # Store reward_scale locally if you plan to use it in the environment's reward function
# # #         # self.reward_scale_val = reward_scale # If you still want to use it manually

# # #         super().__init__(
# # #             robots=robots,
# # #             controller_configs=controller_configs,
# # #             gripper_types=gripper_types,
# # #             initialization_noise=initialization_noise,
# # #             use_camera_obs=use_camera_obs,
# # #             # reward_scale=reward_scale, # Removed from super call
# # #             reward_shaping=reward_shaping, # Keep reward_shaping
# # #             has_renderer=has_renderer,
# # #             has_offscreen_renderer=has_offscreen_renderer,
# # #             render_camera=render_camera,
# # #             render_collision_mesh=render_collision_mesh,
# # #             render_visual_mesh=render_visual_mesh,
# # #             render_gpu_device_id=render_gpu_device_id,
# # #             control_freq=control_freq,
# # #             horizon=horizon,
# # #             ignore_done=ignore_done,
# # #             hard_reset=hard_reset,
# # #             camera_names=camera_names,
# # #             camera_heights=camera_heights,
# # #             camera_widths=camera_widths,
# # #             camera_depths=camera_depths,
# # #         )
# # #         # If you need to scale rewards, do it in your custom reward() method
# # #         # For example, by multiplying the returned reward by a manually stored scale factor.
# # #         # We'll assume a scale of 1.0 if not otherwise handled.
# # #         self._reward_scale = 1.0 # Default internal scale if needed

# # #     def _load_model(self):
# # #         super()._load_model() 

# # #         self.mujoco_arena = TableArena(
# # #             table_full_size=self.table_full_size, 
# # #             table_offset=self.table_offset 
# # #         )

# # #         if self.mujoco_arena.has_worldbody:
# # #              self.model.worldbody.append(self.mujoco_arena)
# # #         else:
# # #              self.model.merge(self.mujoco_arena)


# # #         self.blocks = [] 
# # #         block_size_array = np.array([self.block_half_size] * 3) 

# # #         for i in range(self.num_red_blocks):
# # #             block = BoxObject(
# # #                 name=self.red_block_names[i],
# # #                 size_min=block_size_array, 
# # #                 size_max=block_size_array,
# # #                 rgba=[1, 0, 0, 1], 
# # #                 density=100.,      
# # #                 friction=1.0,      
# # #                 solimp=[0.99,0.99,0.01], 
# # #                 solref=[0.01,1.]         
# # #             )
# # #             self.blocks.append(block)
# # #             self.model.add_object(block) 

# # #         for i in range(self.num_green_blocks):
# # #             block = BoxObject(
# # #                 name=self.green_block_names[i],
# # #                 size_min=block_size_array,
# # #                 size_max=block_size_array,
# # #                 rgba=[0, 1, 0, 1], 
# # #                 density=100., friction=1.0,
# # #                 solimp=[0.99,0.99,0.01], solref=[0.01,1.]
# # #             )
# # #             self.blocks.append(block)
# # #             self.model.add_object(block)
        
# # #         self.placement_initializer = UniformRandomSampler(
# # #             name="ObjectSampler",
# # #             mujoco_objects=self.blocks, 
# # #             x_range=[-0.2, 0.2], 
# # #             y_range=[-0.2, 0.2],    
# # #             rotation=None,            
# # #             rotation_axis='z',        
# # #             ensure_object_boundary_in_range=False, 
# # #             ensure_valid_placement=True, 
# # #             reference_pos=self.table_offset, 
# # #             z_offset=self.block_half_size + 0.001, 
# # #         )
    
# # #     def _setup_references(self):
# # #         super()._setup_references() 

# # #         try:
# # #             self.table_body_id = self.sim.model.body_name2id(self.mujoco_arena.table_body.name) 
# # #         except ValueError:
# # #              try:
# # #                 self.table_body_id = self.sim.model.body_name2id("table") 
# # #                 print("Found table body ID using name: 'table'")
# # #              except ValueError:
# # #                 print(f"Critical Warning: Could not find body ID for the table from TableArena. This might be an issue if table interactions are complex.")
# # #                 self.table_body_id = -1 


# # #         self.block_body_ids = OrderedDict()
# # #         for block_name in self.all_block_names:
# # #             if block_name in self.model.mujoco_objects: 
# # #                 block_mj_obj = self.model.mujoco_objects[block_name]
# # #                 self.block_body_ids[block_name] = self.sim.model.body_name2id(block_mj_obj.root_body)
# # #             else:
# # #                 print(f"Warning: Block {block_name} not found in model.mujoco_objects during _setup_references.")

# # #     def _setup_observables(self):
# # #         observables = super()._setup_observables() 
        
# # #         pf = self.robots[0].robot_model.naming_prefix 
# # #         modality = "object" 

# # #         for block_name in self.all_block_names:
# # #             if block_name not in self.block_body_ids or self.block_body_ids[block_name] == -1: 
# # #                 print(f"Warning: No valid body ID for {block_name} in _setup_observables. Skipping its observables.")
# # #                 continue

# # #             def create_block_sensors(b_name, b_id): 
# # #                 @sensor(modality=modality)
# # #                 def block_pos(obs_cache): 
# # #                     return np.array(self.sim.data.body_xpos[b_id])

# # #                 @sensor(modality=modality)
# # #                 def block_quat(obs_cache):
# # #                     return T.convert_quat(self.sim.data.body_xquat[b_id], to="xyzw")

# # #                 @sensor(modality=modality)
# # #                 def eef_to_block_pos(obs_cache):
# # #                     eef_p = obs_cache.get(f"{pf}eef_pos")
# # #                     if eef_p is None: eef_p = self.robots[0].sim.data.get_body_xpos(self.robots[0].robot_model.eef_name).copy()
# # #                     block_p = np.array(self.sim.data.body_xpos[b_id])
# # #                     if eef_p is not None and block_p is not None:
# # #                         return block_p - eef_p
# # #                     return np.zeros(3) 

# # #                 return block_pos, block_quat, eef_to_block_pos
            
# # #             current_block_body_id = self.block_body_ids[block_name]
# # #             _block_pos_sensor, _block_quat_sensor, _eef_to_block_pos_sensor = create_block_sensors(block_name, current_block_body_id)
            
# # #             observables[f"{block_name}_pos"] = Observable(name=f"{block_name}_pos", sensor=_block_pos_sensor, sampling_rate=self.control_freq)
# # #             observables[f"{block_name}_quat"] = Observable(name=f"{block_name}_quat", sensor=_block_quat_sensor, sampling_rate=self.control_freq)
            
# # #         return observables

# # #     def _reset_internal(self):
# # #         super()._reset_internal() 
        
# # #         if self.placement_initializer is not None:
# # #             object_placements = self.placement_initializer.sample()

# # #             for obj_name, placement_info in object_placements.items():
# # #                 if obj_name in self.model.mujoco_objects:
# # #                     obj_mjcf = self.model.mujoco_objects[obj_name] 
# # #                     pos, quat, _ = placement_info 
# # #                     final_quat = T.normalize_quat(quat) 
# # #                     if obj_mjcf.joints: 
# # #                         self.sim.data.set_joint_qpos(obj_mjcf.joints[0], np.concatenate([pos, final_quat]))
# # #         else:
# # #             print("Warning: Placement initializer not set up in _reset_internal.")

# # #     def reward(self, action=None):
# # #         # Apply manual scaling if needed
# # #         r = 0.0
# # #         if self._check_success():
# # #             r = 1.0
# # #         return r * self._reward_scale # Use the internal scale factor

# # #     def _check_success(self):
# # #         red_stack_ok = True 
# # #         if self.num_red_blocks > 1:
# # #             for i in range(self.num_red_blocks - 1):
# # #                 bottom_block_name = self.red_block_names[i]
# # #                 top_block_name = self.red_block_names[i+1]
                
# # #                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
# # #                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
# # #                     red_stack_ok = False; break 
# # #                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
# # #                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
# # #                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95: 
# # #                     red_stack_ok = False; break
# # #                 block_full_height = self.block_half_size * 2 
# # #                 expected_z_diff = block_full_height
# # #                 actual_z_diff = top_pos[2] - bottom_pos[2]
# # #                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85: 
# # #                     red_stack_ok = False; break
# # #             if not red_stack_ok: return False 

# # #         green_stack_ok = True 
# # #         if self.num_green_blocks > 1:
# # #             for i in range(self.num_green_blocks - 1):
# # #                 bottom_block_name = self.green_block_names[i]
# # #                 top_block_name = self.green_block_names[i+1]
# # #                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
# # #                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
# # #                     green_stack_ok = False; break
# # #                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
# # #                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
# # #                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95:
# # #                     green_stack_ok = False; break
# # #                 block_full_height = self.block_half_size * 2
# # #                 expected_z_diff = block_full_height
# # #                 actual_z_diff = top_pos[2] - bottom_pos[2]
# # #                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85:
# # #                     green_stack_ok = False; break
# # #             if not green_stack_ok: return False 
        
# # #         if self.num_red_blocks <= 1: red_stack_ok = True
# # #         if self.num_green_blocks <= 1: green_stack_ok = True
        
# # #         if self.num_red_blocks == 0 and self.num_green_blocks == 0:
# # #             return False
            
# # #         return red_stack_ok and green_stack_ok

# # #     def visualize(self, vis_settings):
# # #         super().visualize(vis_settings)

# # from collections import OrderedDict
# # import numpy as np
# # from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
# # from robosuite.models.objects import BoxObject
# # from robosuite.models.arenas import TableArena 
# # from robosuite.utils.placement_samplers import UniformRandomSampler
# # from robosuite.utils.observables import Observable, sensor
# # import robosuite.utils.transform_utils as T 

# # class CustomMultiStackEnv(ManipulationEnv):
# #     """
# #     A custom Robosuite environment for stacking multiple red blocks and multiple green blocks.
# #     Uses TableArena for the workspace.
# #     """
# #     def __init__(
# #         self,
# #         robots, 
# #         controller_configs=None, 
# #         gripper_types="default", 
# #         initialization_noise=None, 
# #         table_full_size=(0.8, 0.8, 0.05), 
# #         use_camera_obs=False, 
# #         # reward_shaping=False, # This parameter is removed
# #         has_renderer=False, 
# #         has_offscreen_renderer=True, 
# #         render_camera="frontview", 
# #         render_collision_mesh=False, 
# #         render_visual_mesh=True, 
# #         render_gpu_device_id=-1, 
# #         control_freq=20, 
# #         horizon=1000, 
# #         ignore_done=False, 
# #         hard_reset=True, 
# #         camera_names="agentview", 
# #         camera_heights=256, 
# #         camera_widths=256, 
# #         camera_depths=False, 
# #         num_red_blocks=2, 
# #         num_green_blocks=2, 
# #         block_size=0.022, 
# #     ):
# #         self.num_red_blocks = num_red_blocks
# #         self.num_green_blocks = num_green_blocks
# #         self.block_half_size = block_size 

# #         self.red_block_names = [f"red_cube_{i}" for i in range(num_red_blocks)]
# #         self.green_block_names = [f"green_cube_{i}" for i in range(num_green_blocks)]
# #         self.all_block_names = self.red_block_names + self.green_block_names

# #         self.placement_initializer = None
        
# #         self.table_offset = np.array((0, 0, 0.8)) 
# #         self.table_full_size = table_full_size
        
# #         self._reward_scale = 1.0 # Default internal scale

# #         super().__init__(
# #             robots=robots,
# #             controller_configs=controller_configs,
# #             gripper_types=gripper_types,
# #             initialization_noise=initialization_noise,
# #             use_camera_obs=use_camera_obs,
# #             # reward_shaping=reward_shaping, # Removed from super call
# #             has_renderer=has_renderer,
# #             has_offscreen_renderer=has_offscreen_renderer,
# #             render_camera=render_camera,
# #             render_collision_mesh=render_collision_mesh,
# #             render_visual_mesh=render_visual_mesh,
# #             render_gpu_device_id=render_gpu_device_id,
# #             control_freq=control_freq,
# #             horizon=horizon,
# #             ignore_done=ignore_done,
# #             hard_reset=hard_reset,
# #             camera_names=camera_names,
# #             camera_heights=camera_heights,
# #             camera_widths=camera_widths,
# #             camera_depths=camera_depths,
# #         )

# #     def _load_model(self):
# #         super()._load_model() 

# #         self.mujoco_arena = TableArena(
# #             table_full_size=self.table_full_size, 
# #             table_offset=self.table_offset 
# #         )

# #         if self.mujoco_arena.has_worldbody:
# #              self.model.worldbody.append(self.mujoco_arena)
# #         else:
# #              self.model.merge(self.mujoco_arena)


# #         self.blocks = [] 
# #         block_size_array = np.array([self.block_half_size] * 3) 

# #         for i in range(self.num_red_blocks):
# #             block = BoxObject(
# #                 name=self.red_block_names[i],
# #                 size_min=block_size_array, 
# #                 size_max=block_size_array,
# #                 rgba=[1, 0, 0, 1], 
# #                 density=100.,      
# #                 friction=1.0,      
# #                 solimp=[0.99,0.99,0.01], 
# #                 solref=[0.01,1.]         
# #             )
# #             self.blocks.append(block)
# #             self.model.add_object(block) 

# #         for i in range(self.num_green_blocks):
# #             block = BoxObject(
# #                 name=self.green_block_names[i],
# #                 size_min=block_size_array,
# #                 size_max=block_size_array,
# #                 rgba=[0, 1, 0, 1], 
# #                 density=100., friction=1.0,
# #                 solimp=[0.99,0.99,0.01], solref=[0.01,1.]
# #             )
# #             self.blocks.append(block)
# #             self.model.add_object(block)
        
# #         self.placement_initializer = UniformRandomSampler(
# #             name="ObjectSampler",
# #             mujoco_objects=self.blocks, 
# #             x_range=[-0.2, 0.2], 
# #             y_range=[-0.2, 0.2],    
# #             rotation=None,            
# #             rotation_axis='z',        
# #             ensure_object_boundary_in_range=False, 
# #             ensure_valid_placement=True, 
# #             reference_pos=self.table_offset, 
# #             z_offset=self.block_half_size + 0.001, 
# #         )
    
# #     def _setup_references(self):
# #         super()._setup_references() 

# #         try:
# #             self.table_body_id = self.sim.model.body_name2id(self.mujoco_arena.table_body.name) 
# #         except ValueError:
# #              try:
# #                 self.table_body_id = self.sim.model.body_name2id("table") 
# #                 print("Found table body ID using name: 'table'")
# #              except ValueError:
# #                 print(f"Critical Warning: Could not find body ID for the table from TableArena. This might be an issue if table interactions are complex.")
# #                 self.table_body_id = -1 


# #         self.block_body_ids = OrderedDict()
# #         for block_name in self.all_block_names:
# #             if block_name in self.model.mujoco_objects: 
# #                 block_mj_obj = self.model.mujoco_objects[block_name]
# #                 self.block_body_ids[block_name] = self.sim.model.body_name2id(block_mj_obj.root_body)
# #             else:
# #                 print(f"Warning: Block {block_name} not found in model.mujoco_objects during _setup_references.")

# #     def _setup_observables(self):
# #         observables = super()._setup_observables() 
        
# #         pf = self.robots[0].robot_model.naming_prefix 
# #         modality = "object" 

# #         for block_name in self.all_block_names:
# #             if block_name not in self.block_body_ids or self.block_body_ids[block_name] == -1: 
# #                 print(f"Warning: No valid body ID for {block_name} in _setup_observables. Skipping its observables.")
# #                 continue

# #             def create_block_sensors(b_name, b_id): 
# #                 @sensor(modality=modality)
# #                 def block_pos(obs_cache): 
# #                     return np.array(self.sim.data.body_xpos[b_id])

# #                 @sensor(modality=modality)
# #                 def block_quat(obs_cache):
# #                     return T.convert_quat(self.sim.data.body_xquat[b_id], to="xyzw")

# #                 @sensor(modality=modality)
# #                 def eef_to_block_pos(obs_cache):
# #                     eef_p = obs_cache.get(f"{pf}eef_pos")
# #                     if eef_p is None: eef_p = self.robots[0].sim.data.get_body_xpos(self.robots[0].robot_model.eef_name).copy()
# #                     block_p = np.array(self.sim.data.body_xpos[b_id])
# #                     if eef_p is not None and block_p is not None:
# #                         return block_p - eef_p
# #                     return np.zeros(3) 

# #                 return block_pos, block_quat, eef_to_block_pos
            
# #             current_block_body_id = self.block_body_ids[block_name]
# #             _block_pos_sensor, _block_quat_sensor, _eef_to_block_pos_sensor = create_block_sensors(block_name, current_block_body_id)
            
# #             observables[f"{block_name}_pos"] = Observable(name=f"{block_name}_pos", sensor=_block_pos_sensor, sampling_rate=self.control_freq)
# #             observables[f"{block_name}_quat"] = Observable(name=f"{block_name}_quat", sensor=_block_quat_sensor, sampling_rate=self.control_freq)
            
# #         return observables

# #     def _reset_internal(self):
# #         super()._reset_internal() 
        
# #         if self.placement_initializer is not None:
# #             object_placements = self.placement_initializer.sample()

# #             for obj_name, placement_info in object_placements.items():
# #                 if obj_name in self.model.mujoco_objects:
# #                     obj_mjcf = self.model.mujoco_objects[obj_name] 
# #                     pos, quat, _ = placement_info 
# #                     final_quat = T.normalize_quat(quat) 
# #                     if obj_mjcf.joints: 
# #                         self.sim.data.set_joint_qpos(obj_mjcf.joints[0], np.concatenate([pos, final_quat]))
# #         else:
# #             print("Warning: Placement initializer not set up in _reset_internal.")

# #     def reward(self, action=None):
# #         r = 0.0
# #         if self._check_success():
# #             r = 1.0
# #         return r * self._reward_scale 

# #     def _check_success(self):
# #         red_stack_ok = True 
# #         if self.num_red_blocks > 1:
# #             for i in range(self.num_red_blocks - 1):
# #                 bottom_block_name = self.red_block_names[i]
# #                 top_block_name = self.red_block_names[i+1]
                
# #                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
# #                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
# #                     red_stack_ok = False; break 
# #                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
# #                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
# #                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95: 
# #                     red_stack_ok = False; break
# #                 block_full_height = self.block_half_size * 2 
# #                 expected_z_diff = block_full_height
# #                 actual_z_diff = top_pos[2] - bottom_pos[2]
# #                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85: 
# #                     red_stack_ok = False; break
# #             if not red_stack_ok: return False 

# #         green_stack_ok = True 
# #         if self.num_green_blocks > 1:
# #             for i in range(self.num_green_blocks - 1):
# #                 bottom_block_name = self.green_block_names[i]
# #                 top_block_name = self.green_block_names[i+1]
# #                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
# #                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
# #                     green_stack_ok = False; break
# #                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
# #                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
# #                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95:
# #                     green_stack_ok = False; break
# #                 block_full_height = self.block_half_size * 2
# #                 expected_z_diff = block_full_height
# #                 actual_z_diff = top_pos[2] - bottom_pos[2]
# #                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85:
# #                     green_stack_ok = False; break
# #             if not green_stack_ok: return False 
        
# #         if self.num_red_blocks <= 1: red_stack_ok = True
# #         if self.num_green_blocks <= 1: green_stack_ok = True
        
# #         if self.num_red_blocks == 0 and self.num_green_blocks == 0:
# #             return False
            
# #         return red_stack_ok and green_stack_ok

# #     def visualize(self, vis_settings):
# #         super().visualize(vis_settings)


# from collections import OrderedDict
# import numpy as np
# from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
# from robosuite.models.objects import BoxObject
# from robosuite.models.arenas import TableArena 
# from robosuite.utils.placement_samplers import UniformRandomSampler
# from robosuite.utils.observables import Observable, sensor
# import robosuite.utils.transform_utils as T 

# class CustomMultiStackEnv(ManipulationEnv):
#     """
#     A custom Robosuite environment for stacking multiple red blocks and multiple green blocks.
#     Uses TableArena for the workspace.
#     """
#     def __init__(
#         self,
#         robots, 
#         controller_configs=None, 
#         gripper_types="default", 
#         initialization_noise=None, 
#         table_full_size=(0.8, 0.8, 0.05), 
#         use_camera_obs=False, 
#         # reward_shaping=False, # This parameter is removed
#         has_renderer=False, 
#         has_offscreen_renderer=True, 
#         render_camera="frontview", 
#         render_collision_mesh=False, 
#         render_visual_mesh=True, 
#         render_gpu_device_id=-1, 
#         control_freq=20, 
#         horizon=1000, 
#         ignore_done=False, 
#         hard_reset=True, 
#         camera_names="agentview", 
#         camera_heights=256, 
#         camera_widths=256, 
#         camera_depths=False, 
#         num_red_blocks=2, 
#         num_green_blocks=2, 
#         block_size=0.022, 
#     ):
#         self.num_red_blocks = num_red_blocks
#         self.num_green_blocks = num_green_blocks
#         self.block_half_size = block_size 

#         self.red_block_names = [f"red_cube_{i}" for i in range(num_red_blocks)]
#         self.green_block_names = [f"green_cube_{i}" for i in range(num_green_blocks)]
#         self.all_block_names = self.red_block_names + self.green_block_names

#         self.placement_initializer = None
        
#         self.table_offset = np.array((0, 0, 0.8)) 
#         self.table_full_size = table_full_size
        
#         self._reward_scale = 1.0 # Default internal scale

#         super().__init__(
#             robots=robots,
#             controller_configs=controller_configs,
#             gripper_types=gripper_types,
#             initialization_noise=initialization_noise,
#             use_camera_obs=use_camera_obs,
#             # reward_shaping=reward_shaping, # Removed from super call
#             has_renderer=has_renderer,
#             has_offscreen_renderer=has_offscreen_renderer,
#             render_camera=render_camera,
#             render_collision_mesh=render_collision_mesh,
#             render_visual_mesh=render_visual_mesh,
#             render_gpu_device_id=render_gpu_device_id,
#             control_freq=control_freq,
#             horizon=horizon,
#             ignore_done=ignore_done,
#             hard_reset=hard_reset,
#             camera_names=camera_names,
#             camera_heights=camera_heights,
#             camera_widths=camera_widths,
#             camera_depths=camera_depths,
#         )

#     def _load_model(self):
#         super()._load_model() 

#         self.mujoco_arena = TableArena(
#             table_full_size=self.table_full_size, 
#             table_offset=self.table_offset 
#         )

#         # Directly merge the arena. Most standard arenas are designed for this.
#         self.model.merge_arena(self.mujoco_arena)


#         self.blocks = [] 
#         block_size_array = np.array([self.block_half_size] * 3) 

#         for i in range(self.num_red_blocks):
#             block = BoxObject(
#                 name=self.red_block_names[i],
#                 size_min=block_size_array, 
#                 size_max=block_size_array,
#                 rgba=[1, 0, 0, 1], 
#                 density=100.,      
#                 friction=1.0,      
#                 solimp=[0.99,0.99,0.01], 
#                 solref=[0.01,1.]         
#             )
#             self.blocks.append(block)
#             self.model.add_object(block) 

#         for i in range(self.num_green_blocks):
#             block = BoxObject(
#                 name=self.green_block_names[i],
#                 size_min=block_size_array,
#                 size_max=block_size_array,
#                 rgba=[0, 1, 0, 1], 
#                 density=100., friction=1.0,
#                 solimp=[0.99,0.99,0.01], solref=[0.01,1.]
#             )
#             self.blocks.append(block)
#             self.model.add_object(block)
        
#         self.placement_initializer = UniformRandomSampler(
#             name="ObjectSampler",
#             mujoco_objects=self.blocks, 
#             x_range=[-0.2, 0.2], 
#             y_range=[-0.2, 0.2],    
#             rotation=None,            
#             rotation_axis='z',        
#             ensure_object_boundary_in_range=False, 
#             ensure_valid_placement=True, 
#             reference_pos=self.table_offset, 
#             z_offset=self.block_half_size + 0.001, 
#         )
    
#     def _setup_references(self):
#         super()._setup_references() 

#         try:
#             # TableArena provides a 'table_body' attribute referring to the main table body name
#             self.table_body_id = self.sim.model.body_name2id(self.mujoco_arena.table_body.name) 
#         except Exception as e: # Catch a broader range of exceptions if attribute access fails
#              print(f"Warning: Could not get table_body_id via mujoco_arena.table_body.name (Error: {e}). Trying fallback 'table'.")
#              try:
#                 self.table_body_id = self.sim.model.body_name2id("table") 
#                 print("Found table body ID using name: 'table'")
#              except ValueError:
#                 print(f"Critical Warning: Could not find body ID for the table from TableArena. This might be an issue if table interactions are complex.")
#                 self.table_body_id = -1 


#         self.block_body_ids = OrderedDict()
#         for block_name in self.all_block_names:
#             if block_name in self.model.mujoco_objects: 
#                 block_mj_obj = self.model.mujoco_objects[block_name]
#                 self.block_body_ids[block_name] = self.sim.model.body_name2id(block_mj_obj.root_body)
#             else:
#                 print(f"Warning: Block {block_name} not found in model.mujoco_objects during _setup_references.")

#     def _setup_observables(self):
#         observables = super()._setup_observables() 
        
#         pf = self.robots[0].robot_model.naming_prefix 
#         modality = "object" 

#         for block_name in self.all_block_names:
#             if block_name not in self.block_body_ids or self.block_body_ids[block_name] == -1: 
#                 print(f"Warning: No valid body ID for {block_name} in _setup_observables. Skipping its observables.")
#                 continue

#             def create_block_sensors(b_name, b_id): 
#                 @sensor(modality=modality)
#                 def block_pos(obs_cache): 
#                     return np.array(self.sim.data.body_xpos[b_id])

#                 @sensor(modality=modality)
#                 def block_quat(obs_cache):
#                     return T.convert_quat(self.sim.data.body_xquat[b_id], to="xyzw")

#                 @sensor(modality=modality)
#                 def eef_to_block_pos(obs_cache):
#                     eef_p = obs_cache.get(f"{pf}eef_pos")
#                     if eef_p is None: # Fallback if not in cache (should be, but good to be safe)
#                         eef_p = self.robots[0].sim.data.get_body_xpos(self.robots[0].robot_model.eef_name).copy()
                    
#                     # Get current block_pos directly from sim.data to avoid cache dependency issues for this specific observable
#                     block_p = np.array(self.sim.data.body_xpos[b_id])

#                     if eef_p is not None and block_p is not None: # Should always be true if ids are valid
#                         return block_p - eef_p
#                     return np.zeros(3) # Should not be reached if ids are valid

#                 return block_pos, block_quat, eef_to_block_pos
            
#             current_block_body_id = self.block_body_ids[block_name]
#             _block_pos_sensor, _block_quat_sensor, _eef_to_block_pos_sensor = create_block_sensors(block_name, current_block_body_id)
            
#             observables[f"{block_name}_pos"] = Observable(name=f"{block_name}_pos", sensor=_block_pos_sensor, sampling_rate=self.control_freq)
#             observables[f"{block_name}_quat"] = Observable(name=f"{block_name}_quat", sensor=_block_quat_sensor, sampling_rate=self.control_freq)
#             # eef_to_block_pos can be useful but ensure it doesn't cause issues with observable ordering
#             # For now, let's keep it commented or ensure it's computed in the policy if needed.
#             # observables[f"{pf}eef_to_{block_name}_pos"] = Observable(name=f"{pf}eef_to_{block_name}_pos", sensor=_eef_to_block_pos_sensor, sampling_rate=self.control_freq)

            
#         return observables

#     def _reset_internal(self):
#         super()._reset_internal() 
        
#         if self.placement_initializer is not None:
#             object_placements = self.placement_initializer.sample()

#             for obj_name, placement_info in object_placements.items():
#                 if obj_name in self.model.mujoco_objects:
#                     obj_mjcf = self.model.mujoco_objects[obj_name] 
#                     pos, quat, _ = placement_info 
#                     final_quat = T.normalize_quat(quat) 
#                     if obj_mjcf.joints: 
#                         self.sim.data.set_joint_qpos(obj_mjcf.joints[0], np.concatenate([pos, final_quat]))
#         else:
#             print("Warning: Placement initializer not set up in _reset_internal.")

#     def reward(self, action=None):
#         r = 0.0
#         if self._check_success():
#             r = 1.0
#         return r * self._reward_scale 

#     def _check_success(self):
#         red_stack_ok = True 
#         if self.num_red_blocks > 1:
#             for i in range(self.num_red_blocks - 1):
#                 bottom_block_name = self.red_block_names[i]
#                 top_block_name = self.red_block_names[i+1]
                
#                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
#                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
#                     red_stack_ok = False; break 
#                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
#                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
#                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95: 
#                     red_stack_ok = False; break
#                 block_full_height = self.block_half_size * 2 
#                 expected_z_diff = block_full_height
#                 actual_z_diff = top_pos[2] - bottom_pos[2]
#                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85: 
#                     red_stack_ok = False; break
#             if not red_stack_ok: return False 

#         green_stack_ok = True 
#         if self.num_green_blocks > 1:
#             for i in range(self.num_green_blocks - 1):
#                 bottom_block_name = self.green_block_names[i]
#                 top_block_name = self.green_block_names[i+1]
#                 if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
#                         self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
#                     green_stack_ok = False; break
#                 bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
#                 top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
#                 if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95:
#                     green_stack_ok = False; break
#                 block_full_height = self.block_half_size * 2
#                 expected_z_diff = block_full_height
#                 actual_z_diff = top_pos[2] - bottom_pos[2]
#                 if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85:
#                     green_stack_ok = False; break
#             if not green_stack_ok: return False 
        
#         if self.num_red_blocks <= 1: red_stack_ok = True
#         if self.num_green_blocks <= 1: green_stack_ok = True
        
#         if self.num_red_blocks == 0 and self.num_green_blocks == 0:
#             return False
            
#         return red_stack_ok and green_stack_ok

#     def visualize(self, vis_settings):
#         super().visualize(vis_settings)


from collections import OrderedDict
import numpy as np
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.objects import BoxObject
from robosuite.models.arenas import TableArena 
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.utils.observables import Observable, sensor
import robosuite.utils.transform_utils as T 
from robosuite.models import MujocoModel # Ensure MujocoModel is imported for type checking if needed

class CustomMultiStackEnv(ManipulationEnv):
    """
    A custom Robosuite environment for stacking multiple red blocks and multiple green blocks.
    Uses TableArena for the workspace.
    """
    def __init__(
        self,
        robots, 
        controller_configs=None, 
        gripper_types="default", 
        initialization_noise=None, 
        table_full_size=(0.8, 0.8, 0.05), 
        use_camera_obs=False, 
        has_renderer=False, 
        has_offscreen_renderer=True, 
        render_camera="frontview", 
        render_collision_mesh=False, 
        render_visual_mesh=True, 
        render_gpu_device_id=-1, 
        control_freq=20, 
        horizon=1000, 
        ignore_done=False, 
        hard_reset=True, 
        camera_names="agentview", 
        camera_heights=256, 
        camera_widths=256, 
        camera_depths=False, 
        num_red_blocks=2, 
        num_green_blocks=2, 
        block_size=0.022, 
    ):
        self.num_red_blocks = num_red_blocks
        self.num_green_blocks = num_green_blocks
        self.block_half_size = block_size 

        self.red_block_names = [f"red_cube_{i}" for i in range(num_red_blocks)]
        self.green_block_names = [f"green_cube_{i}" for i in range(num_green_blocks)]
        self.all_block_names = self.red_block_names + self.green_block_names

        self.placement_initializer = None
        
        self.table_offset = np.array((0, 0, 0.8)) 
        self.table_full_size = table_full_size
        
        self._reward_scale = 1.0 # Default internal scale

        super().__init__(
            robots=robots,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
        )

    def _load_model(self):
        # This call should go up the chain (ManipulationEnv -> RobotEnv)
        # RobotEnv._load_model() is responsible for initializing self.model
        super()._load_model() 
        
        # Debug print to check the status of self.model
        print(f"DEBUG: In CustomMultiStackEnv._load_model(), after super()._load_model():")
        print(f"DEBUG: self.model is: {self.model}")
        if self.model is not None:
            print(f"DEBUG: type(self.model) is: {type(self.model)}")

        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size, 
            table_offset=self.table_offset 
        )

        # Explicitly check if self.model is None before trying to use it
        if self.model is None:
            print("CRITICAL ERROR: self.model is None after super()._load_model() call.")
            print("This indicates an issue with the Robosuite superclass model loading.")
            # Attempting to manually initialize self.model as a last resort.
            # This is not standard and suggests a deeper problem if needed.
            print(f"Attempting to manually initialize self.model for robot: {self.robots[0].name}")
            try:
                self.model = MujocoModel(self.robots[0].robot_model.mjcf_path)
                # If we manually initialize, we also need to merge robot controllers.
                # This part of RobotEnv._load_model is also being bypassed.
                for robot in self.robots:
                    robot.load_controller_config(self.controller_configs) # self.controller_configs should be set by RobotEnv.__init__
                    self.model.merge_robot(robot.robot_model)
                print("DEBUG: Manually initialized self.model and merged robot.")
            except Exception as e:
                print(f"CRITICAL ERROR: Manual initialization of self.model failed: {e}")
                raise RuntimeError("Failed to initialize self.model, cannot proceed.")


        # Now, try to merge the arena.
        try:
            self.model.merge_arena(self.mujoco_arena)
        except AttributeError as e:
            print(f"ERROR during merge_arena: {e}")
            print(f"self.model is: {self.model}, type is {type(self.model)}")
            raise e


        self.blocks = [] 
        block_size_array = np.array([self.block_half_size] * 3) 

        for i in range(self.num_red_blocks):
            block = BoxObject(
                name=self.red_block_names[i],
                size_min=block_size_array, 
                size_max=block_size_array,
                rgba=[1, 0, 0, 1], 
                density=100.,      
                friction=1.0,      
                solimp=[0.99,0.99,0.01], 
                solref=[0.01,1.]         
            )
            self.blocks.append(block)
            self.model.add_object(block) 

        for i in range(self.num_green_blocks):
            block = BoxObject(
                name=self.green_block_names[i],
                size_min=block_size_array,
                size_max=block_size_array,
                rgba=[0, 1, 0, 1], 
                density=100., friction=1.0,
                solimp=[0.99,0.99,0.01], solref=[0.01,1.]
            )
            self.blocks.append(block)
            self.model.add_object(block)
        
        self.placement_initializer = UniformRandomSampler(
            name="ObjectSampler",
            mujoco_objects=self.blocks, 
            x_range=[-0.2, 0.2], 
            y_range=[-0.2, 0.2],    
            rotation=None,            
            rotation_axis='z',        
            ensure_object_boundary_in_range=False, 
            ensure_valid_placement=True, 
            reference_pos=self.table_offset, 
            z_offset=self.block_half_size + 0.001, 
        )
    
    def _setup_references(self):
        super()._setup_references() 

        try:
            self.table_body_id = self.sim.model.body_name2id(self.mujoco_arena.table_body.name) 
        except Exception as e: 
             print(f"Warning: Could not get table_body_id via mujoco_arena.table_body.name (Error: {e}). Trying fallback 'table'.")
             try:
                self.table_body_id = self.sim.model.body_name2id("table") 
                print("Found table body ID using name: 'table'")
             except ValueError:
                print(f"Critical Warning: Could not find body ID for the table from TableArena. This might be an issue if table interactions are complex.")
                self.table_body_id = -1 


        self.block_body_ids = OrderedDict()
        for block_name in self.all_block_names:
            if block_name in self.model.mujoco_objects: 
                block_mj_obj = self.model.mujoco_objects[block_name]
                self.block_body_ids[block_name] = self.sim.model.body_name2id(block_mj_obj.root_body)
            else:
                print(f"Warning: Block {block_name} not found in model.mujoco_objects during _setup_references.")

    def _setup_observables(self):
        observables = super()._setup_observables() 
        
        pf = self.robots[0].robot_model.naming_prefix 
        modality = "object" 

        for block_name in self.all_block_names:
            if block_name not in self.block_body_ids or self.block_body_ids[block_name] == -1: 
                print(f"Warning: No valid body ID for {block_name} in _setup_observables. Skipping its observables.")
                continue

            def create_block_sensors(b_name, b_id): 
                @sensor(modality=modality)
                def block_pos(obs_cache): 
                    return np.array(self.sim.data.body_xpos[b_id])

                @sensor(modality=modality)
                def block_quat(obs_cache):
                    return T.convert_quat(self.sim.data.body_xquat[b_id], to="xyzw")

                @sensor(modality=modality)
                def eef_to_block_pos(obs_cache):
                    eef_p = obs_cache.get(f"{pf}eef_pos")
                    if eef_p is None: 
                        eef_p = self.robots[0].sim.data.get_body_xpos(self.robots[0].robot_model.eef_name).copy()
                    
                    block_p = np.array(self.sim.data.body_xpos[b_id])

                    if eef_p is not None and block_p is not None: 
                        return block_p - eef_p
                    return np.zeros(3) 

                return block_pos, block_quat, eef_to_block_pos
            
            current_block_body_id = self.block_body_ids[block_name]
            _block_pos_sensor, _block_quat_sensor, _eef_to_block_pos_sensor = create_block_sensors(block_name, current_block_body_id)
            
            observables[f"{block_name}_pos"] = Observable(name=f"{block_name}_pos", sensor=_block_pos_sensor, sampling_rate=self.control_freq)
            observables[f"{block_name}_quat"] = Observable(name=f"{block_name}_quat", sensor=_block_quat_sensor, sampling_rate=self.control_freq)
            
        return observables

    def _reset_internal(self):
        super()._reset_internal() 
        
        if self.placement_initializer is not None:
            object_placements = self.placement_initializer.sample()

            for obj_name, placement_info in object_placements.items():
                if obj_name in self.model.mujoco_objects:
                    obj_mjcf = self.model.mujoco_objects[obj_name] 
                    pos, quat, _ = placement_info 
                    final_quat = T.normalize_quat(quat) 
                    if obj_mjcf.joints: 
                        self.sim.data.set_joint_qpos(obj_mjcf.joints[0], np.concatenate([pos, final_quat]))
        else:
            print("Warning: Placement initializer not set up in _reset_internal.")

    def reward(self, action=None):
        r = 0.0
        if self._check_success():
            r = 1.0
        return r * self._reward_scale 

    def _check_success(self):
        red_stack_ok = True 
        if self.num_red_blocks > 1:
            for i in range(self.num_red_blocks - 1):
                bottom_block_name = self.red_block_names[i]
                top_block_name = self.red_block_names[i+1]
                
                if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
                        self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
                    red_stack_ok = False; break 
                bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
                top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
                if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95: 
                    red_stack_ok = False; break
                block_full_height = self.block_half_size * 2 
                expected_z_diff = block_full_height
                actual_z_diff = top_pos[2] - bottom_pos[2]
                if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85: 
                    red_stack_ok = False; break
            if not red_stack_ok: return False 

        green_stack_ok = True 
        if self.num_green_blocks > 1:
            for i in range(self.num_green_blocks - 1):
                bottom_block_name = self.green_block_names[i]
                top_block_name = self.green_block_names[i+1]
                if not (bottom_block_name in self.block_body_ids and top_block_name in self.block_body_ids and \
                        self.block_body_ids[bottom_block_name] != -1 and self.block_body_ids[top_block_name] != -1):
                    green_stack_ok = False; break
                bottom_pos = self.sim.data.body_xpos[self.block_body_ids[bottom_block_name]]
                top_pos = self.sim.data.body_xpos[self.block_body_ids[top_block_name]]
                if np.linalg.norm(bottom_pos[:2] - top_pos[:2]) > self.block_half_size * 0.95:
                    green_stack_ok = False; break
                block_full_height = self.block_half_size * 2
                expected_z_diff = block_full_height
                actual_z_diff = top_pos[2] - bottom_pos[2]
                if abs(actual_z_diff - expected_z_diff) > self.block_half_size * 0.85:
                    green_stack_ok = False; break
            if not green_stack_ok: return False 
        
        if self.num_red_blocks <= 1: red_stack_ok = True
        if self.num_green_blocks <= 1: green_stack_ok = True
        
        if self.num_red_blocks == 0 and self.num_green_blocks == 0:
            return False
            
        return red_stack_ok and green_stack_ok

    def visualize(self, vis_settings):
        super().visualize(vis_settings)
