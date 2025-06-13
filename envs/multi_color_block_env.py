import numpy as np

from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.objects import BoxObject
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.models.arenas import TableArena
from robosuite.models.tasks import ManipulationTask


class MultiColorBlockEnv(SingleArmEnv):
    """
    Single-arm env that spawns N coloured blocks at random, collision-free poses
    on the table each reset.
    """

    def __init__(
        self,
        robots="Panda",
        n_blocks: int = 10,
        block_size=(0.02, 0.02, 0.02),
        block_rgba=None,
        # ------------- new table-related kwargs (match Robosuite defaults) -------------
        table_full_size=(0.8, 1.6, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        table_offset=(0.0, 0.0, 0.8),
        base_yaw=0.0,                 # <-- NEW: rotation around z (radians)
        **kwargs,
    ):
        self.n_blocks = n_blocks
        self.block_size = block_size
        if block_rgba is None:
            block_rgba = [
                [0.90, 0.10, 0.10, 1],  # red
                [0.10, 0.90, 0.10, 1],  # green
                [0.10, 0.10, 0.90, 1],  # blue
                [0.90, 0.90, 0.10, 1],  # yellow
                [0.90, 0.10, 0.90, 1],  # magenta
            ]
        self.block_rgba = block_rgba

        self.blocks = []          # BoxObject handles
        self.block_sampler = None  # will be built in _load_model()

        # Store table parameters so _load_model() can access them
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = table_offset

        self.base_yaw = base_yaw

        # All other common kwargs (controller_configs, horizon, etc.) are passed up
        super().__init__(robots=robots, **kwargs)

    # ------------------------------------------------------------------ #
    #  ONE-TIME MODEL BUILD                                              #
    # ------------------------------------------------------------------ #
    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        # Start from a clean slate each time this method is invoked
        # (env construction + every reset when hard_reset=True).
        self.blocks = []

        super()._load_model()

        # rotate + position robot
        yaw = np.array((0, 0, self.base_yaw))
        self.robots[0].robot_model.set_base_ori(yaw)

        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        # load model for table top workspace
        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])

        # ---------- create block assets ----------
        for i in range(self.n_blocks):
            blk = BoxObject(
                name=f"block_{i}",
                size=self.block_size,
                rgba=self.block_rgba[i % len(self.block_rgba)],
            )
            self.blocks.append(blk)

        # ---------- placement sampler (keeps blocks on table) ----------
        half_x, half_y = self.table_full_size[0] / 2, self.table_full_size[1] / 2
        self.block_sampler = UniformRandomSampler(
            name="block_sampler",
            x_range=[-half_x + 0.04, half_x - 0.04],
            y_range=[-half_y + 0.04, half_y - 0.04],
            rotation=None,
            ensure_object_boundary_in_range=True,
            ensure_valid_placement=True,
            reference_pos=self.table_offset,
            z_offset=self.block_size[2] / 2,
        )
        self.block_sampler.reset()
        self.block_sampler.add_objects(self.blocks)

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.blocks,
        )

    # ------------------------------------------------------------------ #
    #  PER-EPISODE RESET                                                 #
    # ------------------------------------------------------------------ #
    def _reset_internal(self):
        super()._reset_internal()                  # resets robot + sim

        # Sample poses for each block
        object_placements = self.block_sampler.sample()

        for obj_pos, obj_quat, obj in object_placements.values():
            qpos = np.concatenate([obj_pos, obj_quat])
            # every BoxObject defines its joint name(s) in obj.joints
            self.sim.data.set_joint_qpos(obj.joints[0], qpos)

        self.sim.forward()                         # commit poses

    # ------------------------------------------------------------------ #
    #  REWARD                                                            #
    # ------------------------------------------------------------------ #
    def reward(self, action=None):
        """
        Very simple placeholder reward.
        Returns 0 every step.  Extend this function with a meaningful
        task-specific reward when you decide what the agent should do.
        """
        return 0.0
