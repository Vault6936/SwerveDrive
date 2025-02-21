package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.MotorDirection;

public class CoralHozCommand extends Command {
    /*
    Manually moves the Coral Placer Horizontally based on a "While True" Button
     */
    CoralSubsystem subsystem;
    MotorDirection direction;
    public CoralHozCommand(CoralSubsystem subsystem, MotorDirection direction){
        this.subsystem = subsystem;
        this.direction = direction;
    }
    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        subsystem.setHozCoral(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setHozCoral(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
