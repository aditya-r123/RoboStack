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
        n_blocks: int = 5,
        block_size=(0.02, 0.02, 0.02),
        block_rgba=None,
        # ------------- new table-related kwargs (match Robosuite defaults) -------------
        table_full_size=(0.8, 1.6, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        table_offset=(0.0, 0.0, 0.8),
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

        # All other common kwargs (controller_configs, horizon, etc.) are passed up
        super().__init__(robots=robots, **kwargs)

    # ------------------------------------------------------------------ #
    #  ONE-TIME MODEL BUILD                                              #
    # ------------------------------------------------------------------ #
    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        super()._load_model()

        # Adjust base pose accordingly
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
            reference_pos=self.table_offset,                    # table origin
            z_offset=self.block_size[2] / 2,
        )

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

        placements = self.block_sampler.sample(self.blocks)
        for obj, (pos, quat) in zip(self.blocks, placements):
            qpos = np.concatenate([pos, quat])
            self.sim.data.set_joint_qpos(f"{obj.name}:joint", qpos)

        self.sim.forward()                         # commit poses
