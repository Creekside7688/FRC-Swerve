package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends CommandBase {
    private final SwerveDrive swerveDrive;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> tSpeedSupplier;

    private final Supplier<Boolean> fieldOriented;
    private final boolean rateLimit;

    public Drive(SwerveDrive swerveDrive, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> tSpeedSupplier,
            Supplier<Boolean> fieldOriented, boolean rateLimit) {
        this.swerveDrive = swerveDrive;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.tSpeedSupplier = tSpeedSupplier;
        this.fieldOriented = fieldOriented;
        this.rateLimit = rateLimit;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = this.xSpeedSupplier.get();
        double ySpeed = this.ySpeedSupplier.get();
        double tSpeed = this.tSpeedSupplier.get();

        // https://www.desmos.com/calculator/ivg5yr9pdy
        xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.DEADBAND);
        ySpeed = MathUtil.applyDeadband(ySpeed, OperatorConstants.DEADBAND);
        tSpeed = MathUtil.applyDeadband(tSpeed, OperatorConstants.DEADBAND);

        xSpeed = Math.pow(xSpeed, 5);
        ySpeed = Math.pow(ySpeed, 5);
        tSpeed = Math.pow(tSpeed, 5);

        xSpeed += (Math.abs(xSpeed) != 0 ? Math.signum(xSpeed) * OperatorConstants.OFFSET : 0.0);
        ySpeed += (Math.abs(ySpeed) != 0 ? Math.signum(ySpeed) * OperatorConstants.OFFSET : 0.0);
        tSpeed += (Math.abs(tSpeed) != 0 ? Math.signum(tSpeed) * OperatorConstants.OFFSET : 0.0);

        swerveDrive.drive(xSpeed, ySpeed, tSpeed, fieldOriented.get(), rateLimit);
    }
}
