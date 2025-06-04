import robosuite as suite
from robosuite.wrappers import VisualizationWrapper 

# Import custom environment and policy
# Ensure these files are in the same directory or Python path
from custom_multi_stack_env import CustomMultiStackEnv 
from multi_stack_policy import MultiStackPolicy     
# pid.py is imported by multi_stack_policy.py

def run_simulation():
    # --- Simulation Parameters ---
    num_episodes = 5
    render_simulation = True  # Set to True to see the simulation, False for headless
    
    # --- Environment Configuration ---
    robot_name = "Panda"  # Available robots: Panda, Sawyer, IIWA, Jaco, UR5e, Baxter
    control_freq_hz = 30  # Control frequency of the environment (affects policy's dt)
    sim_horizon = 3500    # Max steps per episode (increased for potentially complex tasks)

    # --- Custom Environment Parameters ---
    num_r_blocks = 2 # Number of red blocks
    num_g_blocks = 2 # Number of green blocks
    block_hs = 0.025 # Block half-size (e.g., 0.025m for a 5cm cube)

    # Create the custom environment instance
    env = CustomMultiStackEnv(
        robots=robot_name,
        has_renderer=render_simulation,
        has_offscreen_renderer=not render_simulation, 
        use_camera_obs=False,       # Object observations are implicitly handled by sensors
        # use_object_obs=True,      # This line was removed
        control_freq=control_freq_hz,
        horizon=sim_horizon,
        num_red_blocks=num_r_blocks,
        num_green_blocks=num_g_blocks,
        block_size=block_hs,
        table_full_size=(0.7, 0.7, 0.05), 
        initialization_noise={'magnitude': 0.02, 'type': 'gaussian'} 
    )
    
    # Optional: Wrap with VisualizationWrapper for more rendering options if needed
    # if render_simulation:
    #     env = VisualizationWrapper(env, "frontview") # Example camera view

    # --- Simulation Loop ---
    for i_episode in range(num_episodes):
        print(f"--- Starting Episode {i_episode + 1} ---")
        obs = env.reset()
        
        # Initialize the policy with the first observation and environment dt
        policy_control_dt = 1.0 / control_freq_hz
        policy = MultiStackPolicy(
            obs, 
            num_red_blocks=num_r_blocks, 
            num_green_blocks=num_g_blocks, 
            block_half_size=block_hs,
            control_dt=policy_control_dt
        )
        
        done_from_env = False 
        current_step = 0
        total_reward_this_episode = 0

        while not done_from_env and current_step < sim_horizon:
            action = policy.get_action(obs)
            obs, reward, done_from_env, info = env.step(action) 
            total_reward_this_episode += reward
            
            if render_simulation:
                env.render()
            
            if reward > 0: 
                print(f"Episode {i_episode+1}, Step {current_step}: Achieved reward {reward}!")
                if done_from_env: 
                    print(f"Episode {i_episode+1}, Step {current_step}: Environment signaled DONE due to reward/success.")
                    break 
            
            if policy.phase == "ALL_DONE": 
                print(f"Episode {i_episode+1}, Step {current_step}: Policy reached ALL_DONE state.")
                break 

            current_step += 1
            if current_step % 100 == 0: 
                print(f"Episode {i_episode+1}, Step {current_step}: Policy Phase: {policy.phase}, Sub-phase: {policy.sub_phase}, Target Block: {policy.current_target_block_name}, Color: {policy.current_task_color}, Block Idx: {policy.current_block_to_move_idx_in_color_list}")

        print(f"--- Episode {i_episode + 1} Finished ---")
        print(f"Steps taken: {current_step}, Total Reward: {total_reward_this_episode}, Done flag from env: {done_from_env}")
        
        if env._check_success(): 
            print("Environment confirms: SUCCESSFUL STACKING!")
        else:
            print("Environment confirms: Stacking NOT fully successful according to criteria.")
        print("-" * 30)

    # Close the environment
    env.close()
    print("Simulation finished.")

if __name__ == "__main__":
    run_simulation()
