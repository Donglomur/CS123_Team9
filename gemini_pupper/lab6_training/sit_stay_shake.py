"""Lab 6, Step 2: sit, stay, shake.

You get two finished motions:
  whole_sit()   sit down, wait 2 s, stand up (the worked example, sit.py)
  shake_left()  while sitting, shake the left paw

You turn them into five clips the robot can play one at a time:
  sit_down -> sit_idle (repeats, so the dog stays) -> shake_left or shake_right
  (both go back to sit_idle) -> stand_up

Fill in the TODOs, then run:
  uv run python -m mjlab.tasks.tracking.config.pupper.tricks.sit_stay_shake
It checks your code and writes sit_stay_shake.npz, the file you train on.

A Motion is three arrays, one row per frame (50 frames per second):
  root_pos   (T, 3)   where the body is, in metres (x forward, y left, z up)
  root_quat  (T, 4)   how the body is turned, as a quaternion (w, x, y, z)
  joint_pos  (T, 12)  the 12 joint angles, in md.JOINT_NAMES order
Frame i is at time i / 50 seconds.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.tricks import sit

HERE = Path(__file__).resolve().parent
FPS = 50.0

# In whole_sit(), Pupper is sitting from 3.7 s to 5.7 s.
SITTING_FROM_S = 3.7
SITTING_UNTIL_S = 5.7

# Settings for shake_left().
PAW_FORWARD = 0.07  # m
PAW_HEIGHT = 0.10  # m (foot ball centre)
SHAKE = 0.03  # m
LEAN_DEG = 5.0  # roll onto the supporting front leg before lifting the paw


# --------------------------------------------------------------------- provided
def whole_sit() -> md.Motion:
  """The worked example: sit down, hold, stand up."""
  return md.build_motion(sit.keyframes(), fps=FPS)


def sitting_pose():
  """The sitting pose: body position, body angles (degrees) and foot positions."""
  stand = md.stand_feet()
  fr, fl, br, bl = (stand[leg] for leg in md.LEGS)
  feet = {
    "front_r": fr,
    "front_l": fl,
    "back_r": (sit.HIND_X, br[1], md.FOOT_RADIUS),
    "back_l": (sit.HIND_X, bl[1], md.FOOT_RADIUS),
  }
  rpy = (0.0, sit.SIT_PITCH, 0.0)
  root = md.settle_on_ground(
    md.Keyframe(
      0.0, md.root_with_point_on_ground(md.BUTT, rpy, x=sit.BUTT_X), rpy, feet=feet
    )
  ).root_pos
  return root, rpy, feet


def shake_left() -> md.Motion:
  """From sitting: lean right, offer the left paw, shake it twice, sit again."""
  root, rpy, feet = sitting_pose()
  lean_rpy = (LEAN_DEG, rpy[1], rpy[2])  # positive roll lifts the left side
  lean_root = (root[0], root[1] - 0.01, root[2])
  down = feet["front_l"]
  up = (down[0] + PAW_FORWARD, down[1], PAW_HEIGHT)
  low = (up[0], up[1], PAW_HEIGHT - SHAKE)

  def paw_at(p):
    return {**feet, "front_l": p}

  return md.build_motion(
    [
      md.Keyframe(0.0, root, rpy, feet=feet),
      md.Keyframe(0.6, lean_root, lean_rpy, feet=feet),
      md.Keyframe(1.2, lean_root, lean_rpy, feet=paw_at(up)),
      md.Keyframe(1.5, lean_root, lean_rpy, feet=paw_at(low)),
      md.Keyframe(1.8, lean_root, lean_rpy, feet=paw_at(up)),
      md.Keyframe(2.1, lean_root, lean_rpy, feet=paw_at(low)),
      md.Keyframe(2.4, lean_root, lean_rpy, feet=paw_at(up)),
      md.Keyframe(3.0, lean_root, lean_rpy, feet=feet),
      md.Keyframe(3.5, root, rpy, feet=feet),
    ],
    fps=FPS,
  )


# -------------------------------------------------------------------- your code
def cut(motion: md.Motion, start_s: float, end_s: float) -> md.Motion:
  """The piece of ``motion`` from start_s to end_s. Keep both end frames.

  Keeping both ends means two clips cut at the same time share a frame, so
  they join up with no jump.
  """
  # ========== YOUR CODE HERE (TODO 1) ==========
  last = len(motion.root_pos) - 1
  i0 = min(max(round(start_s * motion.fps), 0), last)
  i1 = min(max(round(end_s * motion.fps), i0), last)
  rows = slice(i0, i1 + 1)  # both end frames kept
  return md.Motion(
    fps=motion.fps,
    root_pos=motion.root_pos[rows].copy(),
    root_quat=motion.root_quat[rows].copy(),
    joint_pos=motion.joint_pos[rows].copy(),
  )


def hold(motion: md.Motion, at_s: float, seconds: float) -> md.Motion:
  """Stand still in the pose ``motion`` has at time at_s, for ``seconds``.

  This is the "stay".
  """
  # ========== YOUR CODE HERE (TODO 2) ==========
  i = min(max(round(at_s * motion.fps), 0), len(motion.root_pos) - 1)
  n = round(seconds * motion.fps) + 1  # starts and ends on that pose
  return md.Motion(
    fps=motion.fps,
    root_pos=np.repeat(motion.root_pos[i : i + 1], n, axis=0),
    root_quat=np.repeat(motion.root_quat[i : i + 1], n, axis=0),
    joint_pos=np.repeat(motion.joint_pos[i : i + 1], n, axis=0),
  )


def mirror(motion: md.Motion) -> md.Motion:
  """The same motion with left and right swapped (shake_left -> shake_right).

  Three changes:
    * body position: flip the sign of y.
    * body quaternion: (w, x, y, z) -> (w, -x, y, -z).
    * joints: each left leg gets the right leg's angles, and the other way round.
  For the joints, look at how Pupper's left and right motors are mounted (your
  lab 2 kinematics helps). Do you copy the angles as they are, or flip some
  signs? Which ones? The checks will tell you if you got it right.
  """
  # ========== YOUR CODE HERE (TODO 3) ==========
  # Left and right motors are mounted as mirror images, so a mirrored pose is the
  # other leg's angles with every sign flipped (abduction, hip and knee). The
  # standing pose shows it: front_r (0.26, 0, -0.52), front_l (-0.26, 0, 0.52).
  names = list(md.JOINT_NAMES)

  def partner(name: str) -> str:
    return name.replace("_l_", "_x_").replace("_r_", "_l_").replace("_x_", "_r_")

  src = [names.index(partner(n)) for n in names]
  root_pos = motion.root_pos.copy()
  root_pos[:, 1] *= -1.0
  w, x, y, z = (motion.root_quat[:, k] for k in range(4))
  return md.Motion(
    fps=motion.fps,
    root_pos=root_pos,
    root_quat=np.stack([w, -x, y, -z], axis=1),
    joint_pos=-motion.joint_pos[:, src],
  )


def clips() -> list[md.Clip]:
  """The five clips: sit_down, sit_idle, shake_left, shake_right, stand_up."""
  sitting = whole_sit()
  # ========== YOUR CODE HERE (TODO 4a) ==========
  left = shake_left()
  return [
    md.Clip("sit_down", cut(sitting, 0.0, SITTING_FROM_S), next="sit_idle"),
    md.Clip("sit_idle", hold(sitting, SITTING_FROM_S, 1.0), loop=True),  # the stay
    md.Clip("shake_left", left, next="sit_idle"),
    md.Clip("shake_right", mirror(left), next="sit_idle"),
    md.Clip("stand_up", cut(sitting, SITTING_UNTIL_S, sitting.duration), next=None),
  ]


# ========== YOUR CODE HERE (TODO 4b) ==========
# Every change the robot can make, each at least once: sit_down -> sit_idle,
# sit_idle -> sit_idle (the stay loops), sit_idle -> each shake and back, and
# sit_idle -> stand_up (the exit clip, played from the stay). The shakes appear
# twice each so the policy sees more of the hardest part.
ORDER: list[str] = [
  "sit_down", "sit_idle", "sit_idle",
  "shake_left", "sit_idle", "shake_right", "sit_idle",
  "shake_left", "sit_idle", "shake_right", "sit_idle",
  "stand_up",
]  # fmt: skip


# ----------------------------------------------------------------------- checks
def check_helpers() -> bool:
  """Checks cut, hold and mirror. Prints ok or FAIL for each check."""
  ok = True

  def report(name, passed, detail=""):
    nonlocal ok
    ok &= passed
    print(
      f"  {'ok  ' if passed else 'FAIL'} {name}" + ("" if passed else f": {detail}")
    )

  s = whole_sit()
  a = cut(s, 0.0, SITTING_FROM_S)
  b = cut(s, SITTING_FROM_S, s.duration)
  report(
    "cut: 0-3.7 s is 3.7 s long",
    abs(a.duration - SITTING_FROM_S) < 1e-9,
    f"{a.duration:.3f} s",
  )
  report(
    "cut: neighbours share their boundary frame",
    np.allclose(a.joint_pos[-1], b.joint_pos[0]),
    "last frame of one != first frame of the next",
  )
  report(
    "cut: keeps every frame once",
    len(a.joint_pos) + len(b.joint_pos) - 1 == len(s.joint_pos),
    f"{len(a.joint_pos)} + {len(b.joint_pos)} frames from {len(s.joint_pos)}",
  )
  h = hold(s, SITTING_FROM_S, 1.0)
  report("hold: 1 s long", abs(h.duration - 1.0) < 1e-9, f"{h.duration:.3f} s")
  report(
    "hold: never moves",
    np.ptp(h.joint_pos, axis=0).max() < 1e-12
    and np.ptp(h.root_pos, axis=0).max() < 1e-12,
  )
  report("hold: at the requested pose", np.allclose(h.joint_pos[0], a.joint_pos[-1]))
  left = shake_left()
  right = mirror(left)
  report(
    "mirror: twice gives the original back",
    np.allclose(mirror(right).joint_pos, left.joint_pos)
    and np.allclose(mirror(right).root_quat, left.root_quat),
  )
  stand_error = np.abs(mirror(cut(s, 0.0, 0.0)).joint_pos - s.joint_pos[:1]).max()
  report(
    "mirror: standing (symmetric) mirrors to itself",
    stand_error < 1e-3,
    f"worst joint error {stand_error:.3f} rad",
  )
  report(
    "mirror: shake_right starts and ends in the sitting pose",
    np.abs(right.joint_pos[[0, -1]] - h.joint_pos[[0, -1]]).max() < 0.05,
    f"{np.abs(right.joint_pos[[0, -1]] - h.joint_pos[[0, -1]]).max():.3f} rad away",
  )
  lifted = [
    n for i, n in enumerate(md.JOINT_NAMES) if np.ptp(right.joint_pos[:, i]) > 0.3
  ]
  report(
    "mirror: shake_right moves the right front leg",
    any("front_r" in n for n in lifted) and not any("front_l" in n for n in lifted),
    f"big moves in {lifted}",
  )
  return ok


def main() -> None:
  print("Checking cut, hold and mirror:")
  if not check_helpers():
    raise SystemExit("Fix the failing checks first.")
  if not ORDER:
    raise SystemExit("TODO 4b: set ORDER, the training sequence.")
  library = md.concat_clips(clips(), order=ORDER)
  for key, value in library.report.items():
    print(f"{key:36} {value}")
  for c in library.clips:
    print(
      f"  clip {c['name']:12} frames {c['start']:4}-{c['end']:4} "
      f"({(c['end'] - c['start']) / FPS:.1f} s) loop={c['loop']} next={c['next']}"
    )
  print("wrote", library.save_npz(HERE / "sit_stay_shake.npz"))


if __name__ == "__main__":
  main()
