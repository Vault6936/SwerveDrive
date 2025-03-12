package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends Command {

    private final DriveSubsystem subsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rot;
    public static boolean isFieldCentric;

    public DriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        subsystem = DriveSubsystem.getInstance();
        this.x = x;
        this.y = y;
        this.rot = rot;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        isFieldCentric=SmartDashboard.getBoolean("Field Centric: ", isFieldCentric);
        subsystem.drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble(), isFieldCentric);
    }
}
