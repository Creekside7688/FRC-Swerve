// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    private final Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    private final SwerveDrive swerveDrive = new SwerveDrive();

    private final Drive drive = new Drive(swerveDrive,
            () -> -controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX(),
            () -> !controller.getLeftBumper().getAsBoolean());

    public RobotContainer() {
        swerveDrive.setDefaultCommand(drive);

        configureBindings();
    }

    private void configureBindings() {
        controller.getDown().onTrue(
                Commands.run(
                        () -> swerveDrive.resetHeading()));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
