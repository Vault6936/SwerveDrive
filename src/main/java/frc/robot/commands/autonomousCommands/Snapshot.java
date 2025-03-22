package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.ChoreoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class Snapshot extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    ChoreoSubsystem choreoSubsystem;
    String startLoc;
    String endLoc;
    public Snapshot(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, ChoreoSubsystem choreoSubsystem, String startLoc, String endLoc){
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.choreoSubsystem = choreoSubsystem;
        this.startLoc = startLoc;
        this.endLoc = endLoc;
    }

    @Override
    public void initialize(){
        if (limelightSubsystem.id != -1) {
            CommandScheduler.getInstance().schedule(choreoSubsystem.resetOdometry(startLoc + endLoc));
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
