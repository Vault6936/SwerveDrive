package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift.LiftSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

public class LiftCommand extends Command {
    LiftSubsystem subsystem;
    MotorDirection direction;

    public LiftCommand(LiftSubsystem system, MotorDirection dir)
    {
        subsystem = system;
        direction = dir;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        subsystem.setExtend(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setExtend(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
