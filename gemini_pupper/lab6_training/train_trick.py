"""Train a Pupper clip trick from a local motion file (no W&B). usage: python train_trick.py NAME MOTION.npz ITERS"""
import sys
import mjlab.tasks  # noqa: F401  register tasks
from mjlab.scripts.train import TrainConfig, launch_training
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
name, motion, iters = sys.argv[1], sys.argv[2], int(sys.argv[3])
TASK = "Mjlab-Tracking-Flat-Pupper-v3"
env_cfg = load_env_cfg(TASK)
env_cfg.scene.num_envs = 4096
env_cfg.commands["motion"].motion_file = motion
agent_cfg = load_rl_cfg(TASK)
agent_cfg.max_iterations = iters
agent_cfg.logger = "tensorboard"
agent_cfg.run_name = name
launch_training(TASK, TrainConfig(env=env_cfg, agent=agent_cfg))
print("TRAIN_COMPLETE", name)
