"""Inspect ik.py's current interpolation without starting ROS.

Run: python debug_triangle.py
Plot the foot path: python debug_triangle.py --plot (requires matplotlib)
"""

import argparse
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def load_interpolation():
    # Execute only the position setup and interpolation method from the saved file.
    source = Path(__file__).with_name("ik.py")
    tree = ast.parse(source.read_text(encoding="utf-8"))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "InverseKinematics"
    )
    methods = {
        node.name: node for node in node_class.body
        if isinstance(node, ast.FunctionDef)
    }
    setup_names = {
        "self.ee_triangle_positions", "center_to_rf_hip",
        "self.current_target", "self.t",
    }
    setup = [
        node for node in methods["__init__"].body
        if isinstance(node, ast.Assign)
        and any(ast.unparse(target) in setup_names for target in node.targets)
    ]
    context = {"self": SimpleNamespace(), "np": np, "math": math}
    module = ast.Module(body=setup + [methods["interpolate_triangle"]], type_ignores=[])
    exec(compile(module, str(source), "exec"), context)
    return context["interpolate_triangle"], context["self"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plot", action="store_true", help="plot the sampled foot path")
    args = parser.parse_args()
    interpolate, leg = load_interpolation()
    np.set_printoptions(precision=5, suppress=True)

    print("Expected output: shape (3,), [x, y, z] in meters.")
    for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
        try:
            result = interpolate(leg, t)
            print(f"t={t:.1f}: value={result}, type={type(result).__name__}, "
                  f"shape={getattr(result, 'shape', None)}")
        except Exception as error:
            print(f"t={t:.1f}: {type(error).__name__}: {error}")

    if args.plot:
        import matplotlib.pyplot as plt

        times = np.linspace(0, 3, 301)
        points = np.array([interpolate(leg, t) for t in times])
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(points[:, 0], points[:, 1], points[:, 2], label="interpolated path")
        targets = leg.ee_triangle_positions
        ax.scatter(targets[:, 0], targets[:, 1], targets[:, 2], label="target vertices")
        ax.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)", title="Foot trajectory")
        ax.legend()
        plt.show()


if __name__ == "__main__":
    main()
