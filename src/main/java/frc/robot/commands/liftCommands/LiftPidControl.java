package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.MotorDirection;

import java.util.function.DoubleSupplier;

public class LiftPidControl extends Command {
    LiftSubsystem subsystem;
    DoubleSupplier vertical;

    public LiftPidControl(LiftSubsystem system, DoubleSupplier verticalUp) {
        subsystem = system;
        vertical = verticalUp;
        addRequirements(system);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        subsystem.updatePos(vertical.getAsDouble());
    }

    @Override
    public void end(boolean isCancelled) {
        subsystem.setExtend(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
