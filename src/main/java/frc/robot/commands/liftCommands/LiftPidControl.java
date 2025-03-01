package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.MotorDirection;

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
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
