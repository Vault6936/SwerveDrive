package frc.robot.commands.algaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.MotorDirection;

public class AlgaeAngleCommand extends Command {
    /*
    Changes the angle of the Algae Placer using "While True" Button in chosen direction
     */

    AlgaeSubsystem subsystem;
    MotorDirection direction;
    public AlgaeAngleCommand(AlgaeSubsystem subsystem, MotorDirection direction){
        this.subsystem = subsystem;
        this.direction = direction;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        subsystem.setAngleAlgae(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setAngleAlgae(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
