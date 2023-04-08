// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    private final Joystick controller = new Joystick(OperatorConstants.CONTROLLER_PORT);

    private final SwerveDrive swerveDrive = new SwerveDrive();

    private final Drive drive = new Drive(swerveDrive,
            () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> controller.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> controller.getRawAxis(XboxController.Axis.kRightX.value),
            () -> !controller.getRawButton(XboxController.Button.kLeftBumper.value));

    private final JoystickButton resetHeading = new JoystickButton(controller, 2);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        resetHeading.onTrue(
                Commands.run(
                        () -> swerveDrive.resetHeading()));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
