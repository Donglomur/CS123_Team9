"""Leg kinematics shared by ik.py and walking.py.

Both files import from here, so FK and IK are written once:

    TODO 1     LegKinematics: paste your four-leg FK from lab 2   (handout Part 1)
    TODO 2-4   inverse_kinematics                                 (handout Part 2)

The fr_leg_fk / fl_leg_fk / br_leg_fk / bl_leg_fk wrappers take a leg's three joint
angles as one array and return the foot position in the body frame (meters). Check
your work without the robot:

    python3 kinematics.py
"""

import numpy as np


class LegKinematics:

    def rotation_x(self, angle):
        # rotation about the x-axis implemented for you
        return np.array(
            [
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1],
            ]
        )

    def rotation_y(self, angle):
        return np.array(
            [
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1],
            ]
        )

    def rotation_z(self, angle):
        return np.array(
            [
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

    def translation(self, x, y, z):
        return np.array(
            [
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1],
            ]
        )

    ######################## Per-leg forward kinematics ########################
    #
    # Each function takes the three joint angles of one leg (hip abduction/adduction,
    # hip flexion/extension, knee) and returns the position of that leg's end effector
    # (the foot) as a 3-vector in the base_link frame.
    #
    # Hip motor (joint 1) positions in base_link, as in the lab diagrams. Each is
    # followed by the 0.039 offset from motor 1 to motor 2, expressed in the frame
    # of motor 1 (see the leg_front_l_1 -> leg_front_l_2 diagram).
    #
    #   leg        x        y
    #   front_l   +0.07500  +0.04450
    #   front_r   +0.07500  -0.04450
    #   back_l    -0.07500  +0.03350
    #   back_r    -0.07500  -0.03350
    #
    # The right legs are mirror images of the left legs. Move each leg by hand and check
    # that its marker follows the foot to verify your transforms.

    def fk_front_left(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        ############# Motor conventions according to slides #########

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = (
            translation(0.07500, 0.04450, 0) @ rotation_x(1.57080) @ rotation_z(-theta1)
        )

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        T_1_2 = translation(0, 0, -0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        T_2_3 = (
            translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(-theta3)
        )

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, -0.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        end_effector_position = T_0_ee[:3, 3]

        return end_effector_position

    def fk_front_right(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = (
            translation(0.07500, -0.04450, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
        )

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        T_2_3 = (
            translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)
        )

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        end_effector_position = T_0_ee[:3, 3]

        return end_effector_position

    def fk_back_left(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        # T_0_1 (base_link to leg_back_l_1)
        T_0_1 = (
            translation(-0.07500, 0.03350, 0)
            @ rotation_x(1.57080)
            @ rotation_z(-theta1)
        )

        # T_1_2 (leg_back_l_1 to leg_back_l_2)
        T_1_2 = translation(0, 0, -0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_back_l_2 to leg_back_l_3)
        T_2_3 = (
            translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(-theta3)
        )

        # T_3_ee (leg_back_l_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, -0.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        end_effector_position = T_0_ee[:3, 3]

        return end_effector_position

    def fk_back_right(self, theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            self.rotation_x,
            self.rotation_y,
            self.rotation_z,
            self.translation,
        )

        # T_0_1 (base_link to leg_back_r_1)
        T_0_1 = (
            translation(-0.07500, -0.03350, 0)
            @ rotation_x(1.57080)
            @ rotation_z(theta1)
        )

        # T_1_2 (leg_back_r_1 to leg_back_r_2)
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_back_r_2 to leg_back_r_3)
        T_2_3 = (
            translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)
        )

        # T_3_ee (leg_back_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        # Compute the final transformation
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # Extract the end-effector position
        end_effector_position = T_0_ee[:3, 3]

        return end_effector_position


_legs = LegKinematics()


def fr_leg_fk(theta):
    return _legs.fk_front_right(*theta)


def fl_leg_fk(theta):
    return _legs.fk_front_left(*theta)


def br_leg_fk(theta):
    return _legs.fk_back_right(*theta)


def bl_leg_fk(theta):
    return _legs.fk_back_left(*theta)


# In joint order: front right, front left, back right, back left (see walking.yaml).
LEG_FK = [fr_leg_fk, fl_leg_fk, br_leg_fk, bl_leg_fk]


def inverse_kinematics(
    leg_fk,
    target_ee,
    initial_guess=(0, 0, 0),
    learning_rate=None,
    max_iterations=None,
    tolerance=None,
):
    """Joint angles that put leg_fk's foot at target_ee, found by gradient descent.

    leg_fk is one of the FK functions above, so the same solver works for every leg.
    """
    ################################################################################################
    # TODO 4: Set default values for learning_rate, max_iterations, and tolerance in the
    # signature above. Tolerance is in meters. walking.py overrides max_iterations and tolerance.
    ################################################################################################

    def cost_function(theta):
        # Compute the cost function and the L1 error vector
        # return the cost (a scalar) and l1 (a vector of size 3)
        ################################################################################################
        # TODO 2: Implement the cost function using leg_fk
        ################################################################################################
        return None, None

    def gradient(theta, epsilon=1e-3):
        # Compute the gradient of the cost function using finite differences
        ################################################################################################
        # TODO 3: Implement the gradient computation
        ################################################################################################
        return

    theta = np.array(initial_guess).astype(np.float64)

    cost_l = []
    for _ in range(max_iterations):
        grad = gradient(theta)

        # Update the theta (parameters) using the gradient and the learning rate
        ################################################################################################
        # TODO 4: Implement the gradient update. Use the cost function you implemented, and use tolerance
        # to determine if IK has converged
        # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster
        # convergence
        ################################################################################################

    # print(f'Cost: {cost_l}') # Use to debug to see if your cost function converges within max_iterations

    return theta


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    names = ["front right", "front left", "back right", "back left"]

    print("Foot positions at the zero pose (compare with lab 2):")
    for name, leg_fk in zip(names, LEG_FK):
        print(f"  {name:12s} {leg_fk(np.zeros(3))}")

    # IK round trip: pick reachable angles, ask IK to find the foot position they give.
    # A few millimeters of error or less means IK is working.
    print("IK round trip (walking.py settings, from the zero pose):")
    goal_angles = np.array([0.1, 0.4, -0.8])
    for name, leg_fk in zip(names, LEG_FK):
        target = leg_fk(goal_angles)
        theta = inverse_kinematics(leg_fk, target, max_iterations=100, tolerance=1e-4)
        error_mm = 1000 * np.linalg.norm(leg_fk(theta) - target)
        print(f"  {name:12s} foot error {error_mm:.2f} mm")
