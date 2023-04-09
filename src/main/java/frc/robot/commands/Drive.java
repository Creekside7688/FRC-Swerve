// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Swerve.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends CommandBase {
    private final SwerveDrive swerveDrive;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> tSpeedSupplier;

    private final Supplier<Boolean> fieldOriented;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter tLimiter;

    public Drive(SwerveDrive swerveDrive, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> tSpeedSupplier, Supplier<Boolean> fieldOriented) {
        this.swerveDrive = swerveDrive;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.tSpeedSupplier = tSpeedSupplier;
        this.fieldOriented = fieldOriented;

        this.xLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ACCELERATION_METRES_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ACCELERATION_METRES_PER_SECOND);
        this.tLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double xSpeed = this.xSpeedSupplier.get();
        double ySpeed = this.ySpeedSupplier.get();
        double tSpeed = this.tSpeedSupplier.get();

        // Something something deadband to protect motors.
        xSpeed = Math.abs(xSpeed) > OperatorConstants.DEAD_BAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.DEAD_BAND ? ySpeed : 0.0;
        tSpeed = Math.abs(tSpeed) > OperatorConstants.DEAD_BAND ? tSpeed : 0.0;

        // Reduce acceleration to make it more controllable.
        xSpeed = xLimiter.calculate(xSpeed) * OperatorConstants.TELEOP_MAXIMUM_SPEED;
        ySpeed = yLimiter.calculate(ySpeed) * OperatorConstants.TELEOP_MAXIMUM_SPEED;
        tSpeed = tLimiter.calculate(tSpeed) * OperatorConstants.TELEOP_MAXIMUM_ANGULAR_SPEED;

        ChassisSpeeds chassisSpeeds;

        // The button is negated in the constructor call, so by default it is field relative.
        if(fieldOriented.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, swerveDrive.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
        }

        SwerveModuleState[] states = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveDrive.setStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
