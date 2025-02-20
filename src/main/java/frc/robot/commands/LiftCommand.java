package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSystem;
import frc.robot.subsystems.MotorDirection;

public class LiftCommand extends Command {
    LiftSystem subsystem;
    MotorDirection direction;

    public LiftCommand(LiftSystem system, MotorDirection dir)
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
