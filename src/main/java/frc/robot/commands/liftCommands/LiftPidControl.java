package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LiftPidControl extends Command {
    LiftSubsystem subsystem;
    DoubleSupplier vertical;
    BooleanSupplier overrideLimit;

    public LiftPidControl(LiftSubsystem system, DoubleSupplier verticalUp, BooleanSupplier override) {
        subsystem = system;
        vertical = verticalUp;
        addRequirements(system);
        this.overrideLimit = override;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        subsystem.updatePos(vertical.getAsDouble(), overrideLimit.getAsBoolean());
        subsystem.doPositionControl();
    }

    @Override
    public void end(boolean isCancelled) {
        subsystem.stopMoveToPos();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
