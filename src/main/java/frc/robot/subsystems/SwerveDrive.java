package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FL_DRIVE_MOTOR,
            DriveConstants.FL_TURN_MOTOR,
            DriveConstants.FL_OFFSET);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FR_DRIVE_MOTOR,
            DriveConstants.FR_TURN_MOTOR,
            DriveConstants.FR_OFFSET);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BL_DRIVE_MOTOR,
            DriveConstants.BL_TURN_MOTOR,
            DriveConstants.BL_OFFSET);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BR_DRIVE_MOTOR,
            DriveConstants.BR_TURN_MOTOR,
            DriveConstants.BR_OFFSET);

    // Gyro.
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    // Slew Rate filters to control acceleration.
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.ROTATION_SLEW_RATE);
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Track robot position with odometry
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DriveConstants.SWERVE_KINEMATICS,
            this.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    public SwerveDrive() {
        this.zeroHeading();
    }

    @Override
    public void periodic() {
        odometry.update(
                this.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        SmartDashboard.putNumber("Robot X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Robot Rotation", odometry.getPoseMeters().getRotation().getDegrees());
    }

    /**
     * Returns the estimated position.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets odometry to a specified pose.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                this.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative, boolean rateLimit) {

        // https://www.desmos.com/calculator/ivg5yr9pdy
        xSpeed = Math.pow(xSpeed, 5);
        ySpeed = Math.pow(ySpeed, 5);
        rSpeed = Math.pow(rSpeed, 5);

        xSpeed += (Math.abs(xSpeed) != 0 ? Math.signum(xSpeed) * OperatorConstants.OFFSET : 0.0);
        ySpeed += (Math.abs(ySpeed) != 0 ? Math.signum(ySpeed) * OperatorConstants.OFFSET : 0.0);
        rSpeed += (Math.abs(rSpeed) != 0 ? Math.signum(rSpeed) * OperatorConstants.OFFSET : 0.0);


        double xSpeedCommand;
        double ySpeedCommand;

        if(rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate based on acceleration estimate
            double directionSlewRate;

            if(currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // Instantaneous if not moving
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDif = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);

            if(angleDif < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if(angleDif > 0.85 * Math.PI) {
                if(currentTranslationMagnitude > 1e-4) {
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }
            previousTime = currentTime;

            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rSpeed);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rSpeed;
        }

        // Convert the commanded speeds into proper units
        double xSpeedDelivered = xSpeedCommand * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        double rotDelivered = currentRotation * DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;

        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, this.getRotation2d())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     * Use for defense or when the robot needs to be stationary.
     */
    public void lockPosition() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * How fast the robot is turning in degrees per second.
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_INVERTED ? -1.0 : 1.0);
    }
}