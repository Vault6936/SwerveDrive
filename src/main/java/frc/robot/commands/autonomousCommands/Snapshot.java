package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class Snapshot extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    public Snapshot(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void initialize(){
        if (limelightSubsystem.id != -1) {
            driveSubsystem.poseReset(limelightSubsystem.fieldLoc);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isCancelled)
    {
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
