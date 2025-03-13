package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleStop extends Command {
    DriveSubsystem driveSubsystem;
    boolean stopped = false;

    public ToggleStop(DriveSubsystem subsystem, boolean stopped){
        driveSubsystem = subsystem;
        this.stopped = stopped;
    }

    @Override
    public void end(boolean isCancelled){
        if (stopped)
        {
            driveSubsystem.slowToStop();
        }
        else
        {
            driveSubsystem.allowMove();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
