// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve.DriveConstants;
import frc.robot.Constants.Swerve.ModuleConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final WPI_CANCoder angleEncoder;
    private final boolean angleEncoderReversed;
    private final double angleEncoderOffsetRadians;

    private final PIDController turnController;

    public SwerveModule(int driveMotor, int turnMotor, boolean driveReversed, boolean turnReversed, int angleEncoder, double angleOffset, boolean angleReversed) {
        this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);

        this.driveMotor.setInverted(driveReversed);
        this.turnMotor.setInverted(turnReversed);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = this.turnMotor.getEncoder();
        this.angleEncoder = new WPI_CANCoder(angleEncoder);

        this.driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATION_TO_METRES);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_RPM_TO_METRES_PER_SECOND);
        this.turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_ROTATION_TO_RADIANS);
        this.turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND);

        this.angleEncoderOffsetRadians = angleOffset;
        this.angleEncoderReversed = angleReversed;
        configurateAngleEncoder();

        this.turnController = new PIDController(ModuleConstants.TURN_KP, 0, 0);
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);

        this.resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocty() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAngleEncoderRadians() {
        double angle = angleEncoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;
        angle -= angleEncoderOffsetRadians;

        return angle * (angleEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(this.getAngleEncoderRadians());
    }

    private void configurateAngleEncoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = false;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        angleEncoder.configAllSettings(config);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocty(), new Rotation2d(this.getTurnPosition()));
    }

    public void setState(SwerveModuleState state) {
        // Prevent the wheels from going back to their default state when there is no input.
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, this.getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);
        turnMotor.set(turnController.calculate(this.getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
