# CS 123 — Team 9

Stanford **CS 123: A Hands-On Introduction to Building AI-Enabled Robots** — Team 9's working repo.
Each lab lives in its own subfolder, imported from the course repos under
[cs123-stanford](https://github.com/cs123-stanford) via `git subtree`.

## Structure

| Folder | Lab | Source |
|--------|-----|--------|
| `pd_control_lab/` | Lab 1 — ROS2 intro + PD control | `cs123-stanford/pd_control_lab` |

## Pulling course updates into a lab subfolder

```bash
git subtree pull --prefix=pd_control_lab \
  https://github.com/cs123-stanford/pd_control_lab.git main --squash
```

## Adding a future lab as a subfolder

```bash
git subtree add --prefix=<folder> \
  https://github.com/cs123-stanford/<repo>.git main --squash
```
