#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <tgmath.h>

// In this file, x is to the right, y is forwards, and z is up.
// The delta robot exists parallel to the xy plane.

typedef struct {
    // The length of the upper (actuated) arm, in meters.
    double arm_length_upper;

    // The length of the lower (passive) segment, in meters.
    double arm_length_lower;

    // The distance from the center of the base to the center of the upper
    // actuated joint, in meters.
    double base_radius;

    // The distance from the center of the manipulator to the center of the
    // lower passive segment, in meters.
    double manipulator_radius;
} delta_robot_parameters;

double dot(
        double x[3],
        double y[3]) {
    return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

bool inverse_kinematics(
        delta_robot_parameters* params,
        double position[3],
        double angles_out[3]) {
    for (int i = 0; i < 3; i++) {
        double theta = 2 * i * M_PI / 3.0;
        double manipulator_joint_pos[3];
        {
            manipulator_joint_pos[0] = position[0]
                + params->manipulator_radius * cos(theta);
            manipulator_joint_pos[1] = position[1]
                + params->manipulator_radius * sin(theta);
            manipulator_joint_pos[2] = position[2];
        }

        double base_joint_pos[3];
        {
            base_joint_pos[0] = params->base_radius * cos(theta);
            base_joint_pos[1] = params->base_radius * sin(theta);
            base_joint_pos[2] = 0;
        }

        // Now we want to intersect a sphere centered at
        // `manipulator_joint_pos` with radius `arm_length_lower` with the
        // circle of radius `arm_length_upper` centered at
        // `base_joint_position` in the plane perpendicular to xy-plane and
        // containing the origin and `base_joint_position`.
        double circle_normal[3];
        {
            circle_normal[0] = -sin(theta);
            circle_normal[1] = cos(theta);
            circle_normal[2] = 0;
        }

        // Find the minimum distance from `manipulator_joint_pos` to the plane
        // with equation:
        // `circle_normal . x = circle_normal . base_joint_pos`
        double dist = dot(circle_normal, manipulator_joint_pos);

        double circle_radius = sqrt(params->arm_length_lower * params->arm_length_lower - dist * dist);

        double circle_center[3];
        {
            circle_center[0] = manipulator_joint_pos[0] - dist * circle_normal[0];
            circle_center[1] = manipulator_joint_pos[1] - dist * circle_normal[1];
            circle_center[2] = manipulator_joint_pos[2] - dist * circle_normal[2];
        }

        double circle_center_offset[3];
        {
            circle_center_offset[0] = base_joint_pos[0] - circle_center[0];
            circle_center_offset[1] = base_joint_pos[1] - circle_center[1];
            circle_center_offset[2] = base_joint_pos[2] - circle_center[2];
        }

        // Now, find the distance between `circle_center` and `base_joint_pos`.
        double center_to_center = sqrt(
                dot(circle_center_offset, circle_center_offset));

        // If the triangle inequality is violated, the problem is ill-defined.
        if (center_to_center > circle_radius + params->arm_length_upper
                || params->arm_length_upper > circle_radius + center_to_center
                || circle_radius > params->arm_length_upper + center_to_center) {
            return false;
        }

        // Find the intersection of two circles lying on the x-axis with
        // `center_to_center` between them and with the given radii.
        // |(x, y)|^2 = (params->arm_length_upper)^2
        //
        // y^2 = (params->arm_length_upper)^2 - x^2
        //
        // |(x - center_to_center, y)|^2 = circle_radius^2
        //
        //  y^2 = circle_radius^2 - x^2 + 2 * x * center_to_center
        //          - center_to_center^2
        //
        //  (params->arm_length_upper)^2 - circle_radius^2 + center_to_center^2
        //      = 2 * x * center_to_center
        double x = ((params->arm_length_upper * params->arm_length_upper)
                - (circle_radius * circle_radius)
                + (center_to_center * center_to_center))
            / (2 * center_to_center);
        double y = -sqrt(
                params->arm_length_upper * params->arm_length_upper - x * x);

        double phi = atan2(y, x);

        double angle_offset_cos = -(
                    circle_center_offset[0] * cos(theta)
                    + circle_center_offset[1] * sin(theta))
                / sqrt(dot(circle_center_offset, circle_center_offset));
        double angle_offset = acos(angle_offset_cos);
        angles_out[i] = phi + angle_offset;
    }

    // If we got this far, we can find angles for each of the arms so the
    // problem is well-defined. Let the user know.
    return true;
}

bool verify(
        delta_robot_parameters* params,
        double position[3],
        double angles[3]) {
    for (int i = 0; i < 3; i++) {
        double theta = 2 * i * M_PI / 3.0;

        double manipulator_joint_pos[3];
        {
            manipulator_joint_pos[0] = position[0]
                + params->manipulator_radius * cos(theta);
            manipulator_joint_pos[1] = position[1]
                + params->manipulator_radius * sin(theta);
            manipulator_joint_pos[2] = position[2];
        }

        double base_joint_pos[3];
        {
            base_joint_pos[0] = params->base_radius * cos(theta);
            base_joint_pos[1] = params->base_radius * sin(theta);
            base_joint_pos[2] = 0;
        }

        double mid_joint_pos[3];
        {
            mid_joint_pos[0] = base_joint_pos[0]
                + params->arm_length_upper * cos(theta) * cos(angles[i]);
            mid_joint_pos[1] = base_joint_pos[1]
                + params->arm_length_upper * sin(theta) * cos(angles[i]);
            mid_joint_pos[2] = base_joint_pos[2]
                - params->arm_length_upper * sin(angles[i]);
        }

        double distance_lower_arm = sqrt(
            pow(mid_joint_pos[0] - manipulator_joint_pos[0], 2)
            + pow(mid_joint_pos[1] - manipulator_joint_pos[1], 2)
            + pow(mid_joint_pos[2] - manipulator_joint_pos[2], 2)
        );

        double error = distance_lower_arm - params->arm_length_lower;
        if (fabs(error) > 1e-6) {
            printf("Error: %f\n", error);
            abort();
        }
    }
}

void test(delta_robot_parameters* params, double x, double y, double z) {
    double position[3];
    {
        position[0] = x;
        position[1] = y;
        position[2] = z;
    }

    double angles[3];
    bool possible = inverse_kinematics(params, position, angles);
    if (possible) {
        /*
        printf("Going to (%f, %f, %f) with angles (%f, %f, %f).\n",
                position[0], position[1], position[2],
                angles[0], angles[1], angles[2]);
                */
        fprintf(stdout, "%f %f %f %f %f %f\n",
                position[0], position[1], position[2],
                angles[0], angles[1], angles[2]);
        verify(params, position, angles);
    } else {
        fprintf(stderr, "Cannot go to position (%f, %f, %f).\n",
                position[0], position[1], position[2]
                );
    }
}

int main() {
    delta_robot_parameters params;
    params.arm_length_upper = 2.854;
    params.arm_length_lower = 4;
    params.base_radius = 2;
    params.manipulator_radius = 1;

    for (int i = 0; i < 25; i++) {
        test(&params, 0, 0, -i * 0.2 - 1);
    }

    for (int i = 0; i < 20; i++) {
        test(&params, 0, 0, -6 + i * 0.1);
    }

    for (int i = 0; i < 10; i++) {
        double t = i * .1;
        test(&params, 2 * t * cos(t), 2 * t * sin(t), -4);
    }

    for (int i = 0; i < 100; i++) {
        double t = i * .1;
        test(&params, 2 * cos(1 + t), 2 * sin(1 + t), -4);
    }

    for (int i = 0; i < 100; i++) {
        double t = i * .1;
        test(&params, 2 * (1 - t / 10) * cos(11 + t), 2 * (1 - t / 10) * sin(11 - t), -4 + 3.0 * t / 10.0);
    }

    test(&params, 0, 0, 0);

    return 0;
}
