package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilAlign extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pid_hoz = new PIDController(0.5,0,0);
    PIDController pid_rot = new PIDController(0.5,0,0);


    public AprilAlign(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        this.limelightSubsystem = limelightSubsystem;
    }
    //     ☆*: .｡. o(≧▽≦)o .｡.:*☆

    @Override
    public void execute() {
        double tx = limelightSubsystem.tx;
        double ry = limelightSubsystem.ry;
        double pidHozMovement = pid_hoz.calculate(tx,0);
        double pidRotationMovement = MathUtil.clamp(pid_rot.calculate(ry,0), -0.5, 0.5);
        if (Constants.DebugInfo.debugAlign)
        {
            SmartDashboard.putNumber("tx", tx);
            SmartDashboard.putNumber("ry", ry);
            SmartDashboard.putNumber("pidHozMovement", pidHozMovement);
            SmartDashboard.putNumber("pidRotationMovement", pidRotationMovement);
        }
        driveSubsystem.drive(-pidHozMovement,0 ,pidRotationMovement);
    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }


    @Override
    public boolean isFinished()
    {
        return limelightSubsystem.id == 0;
    }

}
