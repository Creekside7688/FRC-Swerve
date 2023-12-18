package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.lib.zylve.Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();

    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();

        swerveDrive.setDefaultCommand(
                new RunCommand(
                        () -> swerveDrive.drive(
                                -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                                true, true),
                        swerveDrive));
    }

    private void configureButtonBindings() {
        controller.getRightBumper()
                .whileTrue(new RunCommand(
                        () -> swerveDrive.lockPosition(),
                        swerveDrive));
    }

    public Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // ProfiledPIDController thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        // exampleTrajectory,
        // swerveDrive::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // swerveDrive::setModuleStates,
        // swerveDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false, false));

        return null;
    }
}
