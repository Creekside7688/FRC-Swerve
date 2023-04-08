// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Swerve.DriveConstants;

public final class Constants {
    public static class OperatorConstants {
        public static final int CONTROLLER_PORT = 0;

        public static final double TELEOP_MAX_SPEED = DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND / 4;
        public static final double TELEOP_MAX_ANGULAR_SPEED = DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND
                / 4;

        public static final double DEAD_BAND = 0.05;
    }

    public static class Swerve {
        public static class ModuleConstants {
            public static final double WHEEL_DIAMETER_METRES = Units.inchesToMeters(4);
            public static final double DRIVE_GEAR_RATIO = 1.0 / 8.14;
            public static final double TURN_GEAR_RATIO = 1.0 / (150.0 / 7.0);

            public static final double DRIVE_ENCODER_ROTATION_TO_METRES = DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METRES
                    * Math.PI;
            public static final double DRIVE_ENCODER_RPM_TO_METRES_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METRES / 60;

            public static final double TURN_ENCODER_ROTATION_TO_RADIANS = TURN_GEAR_RATIO * 2 * Math.PI;
            public static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60;

            public static final double TURN_KP = 0.5;
        }

        public static class DriveConstants {
            // Ideally we would build a square chassis so these would be the same.
            public static final double TRACK_WIDTH_METRES = Units.inchesToMeters(28);
            public static final double WHEEL_BASE_METRES = Units.inchesToMeters(28);

            public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                    new Translation2d(WHEEL_BASE_METRES / 2, -TRACK_WIDTH_METRES / 2),
                    new Translation2d(WHEEL_BASE_METRES / 2, TRACK_WIDTH_METRES / 2),
                    new Translation2d(-WHEEL_BASE_METRES / 2, -TRACK_WIDTH_METRES / 2),
                    new Translation2d(-WHEEL_BASE_METRES / 2, TRACK_WIDTH_METRES / 2));

            public static final double MAXIMUM_SPEED_METRES_PER_SECOND = 3.0;
            public static final double MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * 2.0 * Math.PI;

            public static final double MAXIMUM_ACCELERATION_PER_SECOND = 3.0;
            public static final double MAXIMUM_ANGULAR_ACCELERATION_PER_SECOND = 3.0;

            public static final int FL_DRIVE_MOTOR_ID = 1;
            public static final int FR_DRIVE_MOTOR_ID = 1;
            public static final int BL_DRIVE_MOTOR_ID = 1;
            public static final int BR_DRIVE_MOTOR_ID = 1;

            public static final int FL_TURN_MOTOR_ID = 1;
            public static final int FR_TURN_MOTOR_ID = 1;
            public static final int BL_TURN_MOTOR_ID = 1;
            public static final int BR_TURN_MOTOR_ID = 1;

            public static final boolean FL_DRIVE_ENCODER_REVERSED = false;
            public static final boolean FR_DRIVE_ENCODER_REVERSED = false;
            public static final boolean BL_DRIVE_ENCODER_REVERSED = false;
            public static final boolean BR_DRIVE_ENCODER_REVERSED = false;

            public static final boolean FL_TURN_ENCODER_REVERSED = false;
            public static final boolean FR_TURN_ENCODER_REVERSED = false;
            public static final boolean BL_TURN_ENCODER_REVERSED = false;
            public static final boolean BR_TURN_ENCODER_REVERSED = false;

            public static final int FL_ANGLE_ENCODER_ID = 1;
            public static final int FR_ANGLE_ENCODER_ID = 1;
            public static final int BL_ANGLE_ENCODER_ID = 1;
            public static final int BR_ANGLE_ENCODER_ID = 1;

            public static final int FL_ANGLE_ENCODER_OFFSET_RADIANS = 1;
            public static final int FR_ANGLE_ENCODER_OFFSET_RADIANS = 1;
            public static final int BL_ANGLE_ENCODER_OFFSET_RADIANS = 1;
            public static final int BR_ANGLE_ENCODER_OFFSET_RADIANS = 1;

            public static final boolean FL_ANGLE_ENCODER_REVERSED = false;
            public static final boolean FR_ANGLE_ENCODER_REVERSED = false;
            public static final boolean BL_ANGLE_ENCODER_REVERSED = false;
            public static final boolean BR_ANGLE_ENCODER_REVERSED = false;
        }
    }
}