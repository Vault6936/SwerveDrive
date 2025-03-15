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
    double aprilX = 0;
    double aprilRot = 0;


    public AprilAlign(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem);
    }


    @Override
    public void execute() {
        aprilX = limelightSubsystem.tx;
        aprilRot = limelightSubsystem.ry;
        double pidHozMovement = pid_hoz.calculate(aprilX,0);
        double pidRotationMovement = MathUtil.clamp(pid_rot.calculate(aprilRot,0), -0.5, 0.5);
        if (Constants.DebugInfo.debugAlign)
        {
            SmartDashboard.putNumber("tx", aprilX);
            SmartDashboard.putNumber("ry", aprilRot);
            SmartDashboard.putNumber("pidHozMovement", pidHozMovement);
            SmartDashboard.putNumber("pidRotationMovement", pidRotationMovement);
        }
        driveSubsystem.drive(-pidHozMovement,0, pidRotationMovement);
    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }


    @Override
    public boolean isFinished()
    {
        return Math.abs(aprilX) < .1 && Math.abs(aprilRot) < .1;
    }

}
