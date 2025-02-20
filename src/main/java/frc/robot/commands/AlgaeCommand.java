package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEaterSystem;
import frc.robot.subsystems.MotorDirection;

public class AlgaeCommand extends Command {
    AlgaeEaterSystem subsystem;
    MotorDirection direction;

    public AlgaeCommand(AlgaeEaterSystem system, MotorDirection dir)
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
        subsystem.setIntake(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setIntake(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
