package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends Command {

    private final DriveSubsystem subsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rot;
    public static boolean isFieldCentric;

    double lastX = 0;
    double lastY = 0;
    double lastRot = 0;
    final double ACCEL_LIMIT = 0.02;
    final double ROT_LIMIT = 0.02;

    public DriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        subsystem = DriveSubsystem.getInstance();
        this.x = x;
        this.y = y;
        this.rot = rot;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double currentX = x.getAsDouble();
        double currentY = y.getAsDouble();
        double currentRot = rot.getAsDouble();
        lastX = doAcceleration(currentX, lastX, ACCEL_LIMIT);
        lastY = doAcceleration(currentY, lastY, ACCEL_LIMIT);
        lastRot = doAcceleration(currentRot, lastRot, ROT_LIMIT);
        subsystem.drive(lastX, lastY, lastRot, isFieldCentric);
        SmartDashboard.putBoolean("FieldCentric", isFieldCentric);
    }

    public double doAcceleration(double input, double lastSpeed, double limit)
    {
        return input;
//        if(input > 0) {
//            return MathUtil.clamp(input, 0, lastSpeed + limit);
//        }
//        else
//        {
//            return MathUtil.clamp(input, lastSpeed - limit, 0);
//        }
    }
}
