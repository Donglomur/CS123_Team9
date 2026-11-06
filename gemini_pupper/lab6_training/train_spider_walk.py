"""Spider walk (Mjlab-SpiderWalk-Flat-Pupper-v3), Lab 4 StableGait weights adapted to the spider stance. No W&B."""
import mjlab.tasks  # noqa: F401
from mjlab.scripts.train import TrainConfig, launch_training
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
TASK = "Mjlab-SpiderWalk-Flat-Pupper-v3"
STABLE_GAIT = {  # lai-ethan-stanford/mjlab/1e8sy1f5 (Lab 4 final_reference, pupper_stable_gait)
  "track_linear_velocity": 10, "track_yaw_velocity": 8, "upright": 5, "air_time": 5, "base_height": 3,
  "gait_tracking": 5, "track_angular_velocity": 3, "termination": -8, "orientation_l2": -2,
  "lin_vel_z_l2": -2, "ang_vel_xy_l2": 0, "foot_slip": -3, "action_rate_l2": -5, "joint_torques_l2": 0,
  "joint_acc_l2": 0, "stand_still_pose": -1.5, "stand_still_joint_velocity": -1.5,
  "knee_ground_contact": -4, "abduction_angle": -3, "self_collision_l": -5, "self_collision_r": -5,
}
SPIDER = dict(STABLE_GAIT, abduction_angle=0.0, air_time=2.0)  # the splay is the stance; low body: no hops
env_cfg = load_env_cfg(TASK)
for name, weight in SPIDER.items():
  env_cfg.rewards[name].weight = weight
env_cfg.scene.num_envs = 4096
agent_cfg = load_rl_cfg(TASK)
agent_cfg.max_iterations = 4000
agent_cfg.logger = "tensorboard"
agent_cfg.run_name = "spider_walk"
launch_training(TASK, TrainConfig(env=env_cfg, agent=agent_cfg))
print("TRAIN_COMPLETE spider_walk")
