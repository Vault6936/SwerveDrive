package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.MotorDirection;

import java.util.function.DoubleSupplier;

public class CoralHozPidControl extends Command {
    /*
    Don't understand what this class is for.
    This is the default command that just moves the subsystem to a target position.
     */
    CoralSubsystem subsystem;
    DoubleSupplier horizontal;
    public CoralHozPidControl(CoralSubsystem subsystem, DoubleSupplier horizontal){
        this.subsystem = subsystem;
        this.horizontal = horizontal;
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        subsystem.updateHozTarget(horizontal.getAsDouble());
        subsystem.doPositionControl();
    }

    @Override
    public void end(boolean isCancelled)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
