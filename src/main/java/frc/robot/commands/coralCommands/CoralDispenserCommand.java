package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.MotorDirection;

public class CoralDispenserCommand extends Command {
    /*
    Manually moves the Coral dispensers using "While True" on a button
    */
    CoralSubsystem subsystem;
    MotorDirection direction;

    public CoralDispenserCommand(CoralSubsystem system, MotorDirection dir)
    {
        subsystem = system;
        direction = dir;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute()
    {
        subsystem.setDispenser(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setDispenser(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
