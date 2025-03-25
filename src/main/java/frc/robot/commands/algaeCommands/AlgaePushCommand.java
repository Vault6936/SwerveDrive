package frc.robot.commands.algaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

public class AlgaePushCommand extends Command {
    /*
    Manually moves the Algae pushers using "While True" on a button
     */
    AlgaeSubsystem subsystem;
    MotorDirection direction;

    public AlgaePushCommand(AlgaeSubsystem system, MotorDirection dir)
    {
        subsystem = system;
        direction = dir;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        subsystem.setPushAlgae(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setPushAlgae(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
