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

    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> tSpeed;

    private final Supplier<Boolean> fieldOriented;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter tLimiter;

    public Drive(SwerveDrive swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> tSpeed, Supplier<Boolean> fieldOriented) {
        this.swerveDrive = swerveDrive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.tSpeed = tSpeed;
        this.fieldOriented = fieldOriented;

        this.xLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ACCELERATION_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ACCELERATION_PER_SECOND);
        this.tLimiter = new SlewRateLimiter(DriveConstants.MAXIMUM_ANGULAR_ACCELERATION_PER_SECOND);
        
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = this.xSpeed.get();
        double ySpeed = this.ySpeed.get();
        double tSpeed = this.tSpeed.get();

        xSpeed = Math.abs(xSpeed) > OperatorConstants.DEAD_BAND ? xSpeed : 0.0;
        ySpeed = Math.abs(xSpeed) > OperatorConstants.DEAD_BAND ? ySpeed : 0.0;
        tSpeed = Math.abs(xSpeed) > OperatorConstants.DEAD_BAND ? tSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        tSpeed = tLimiter.calculate(tSpeed) * DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds chassisSpeeds;

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
        swerveDrive.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
