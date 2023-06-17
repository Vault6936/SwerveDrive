package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends CommandBase {

    private final DriveSubsystem subsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rot;

    public DriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        subsystem = Robot.robotContainer.driveSubsystem;
        this.x = x;
        this.y = y;
        this.rot = rot;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble());
    }
}
