// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.DriveConstants;

public class SwerveDrive extends SubsystemBase {
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FL_DRIVE_MOTOR_ID,
            DriveConstants.FL_TURN_MOTOR_ID,
            DriveConstants.FL_DRIVE_ENCODER_REVERSED,
            DriveConstants.FL_TURN_ENCODER_REVERSED,
            DriveConstants.FL_ANGLE_ENCODER_ID,
            DriveConstants.FL_ANGLE_ENCODER_OFFSET_RADIANS,
            DriveConstants.FL_ANGLE_ENCODER_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FR_DRIVE_MOTOR_ID,
            DriveConstants.FR_TURN_MOTOR_ID,
            DriveConstants.FR_DRIVE_ENCODER_REVERSED,
            DriveConstants.FR_TURN_ENCODER_REVERSED,
            DriveConstants.FR_ANGLE_ENCODER_ID,
            DriveConstants.FR_ANGLE_ENCODER_OFFSET_RADIANS,
            DriveConstants.FR_ANGLE_ENCODER_REVERSED);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BL_DRIVE_MOTOR_ID,
            DriveConstants.BL_TURN_MOTOR_ID,
            DriveConstants.BL_DRIVE_ENCODER_REVERSED,
            DriveConstants.BL_TURN_ENCODER_REVERSED,
            DriveConstants.BL_ANGLE_ENCODER_ID,
            DriveConstants.BL_ANGLE_ENCODER_OFFSET_RADIANS,
            DriveConstants.BL_ANGLE_ENCODER_REVERSED);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BR_DRIVE_MOTOR_ID,
            DriveConstants.BR_TURN_MOTOR_ID,
            DriveConstants.BR_DRIVE_ENCODER_REVERSED,
            DriveConstants.BR_TURN_ENCODER_REVERSED,
            DriveConstants.BR_ANGLE_ENCODER_ID,
            DriveConstants.BR_ANGLE_ENCODER_OFFSET_RADIANS,
            DriveConstants.BR_ANGLE_ENCODER_REVERSED);

    public SwerveDrive() {
        // Gyro recalibrates on startup so we have to delay the zeroing of it.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                this.resetHeading();
            } catch(Exception e) {}
        }).start();
    }

    public void resetHeading() {
        gyro.reset();
    }

    // Gryo is continuous, so we have to use this weird function to get something between 0 and 360
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getHeading());
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void periodic() {
    }
}
