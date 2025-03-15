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
    PIDController pidStrafe = new PIDController(0.5,0,0);
    PIDController pidRot = new PIDController(0.5,0,0);
    double aprilX; // Meters
    double aprilDist; // Meters
    double aprilRot; // Radians
    double targetDist; // Meters


    public AprilAlign(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double targetDist){
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem);
        aprilX = limelightSubsystem.tx;
        aprilDist = limelightSubsystem.tz;
        aprilRot = limelightSubsystem.ry;
        this.targetDist = targetDist;
    }


    @Override
    public void execute() {
        aprilDist = limelightSubsystem.tz;
        aprilX = limelightSubsystem.tx;
        aprilRot = limelightSubsystem.ry;

        double pidCalcX = pidStrafe.calculate(aprilX, driveSubsystem.currentPose.getX());
        double pidCalcY = pidStrafe.calculate(aprilDist, driveSubsystem.currentPose.getY() + targetDist);
        double pidCalcRot = pidRot.calculate(aprilRot, driveSubsystem.currentPose.getRotation().getRadians());

        pidCalcX = MathUtil.clamp(pidCalcX, -1,1);
        pidCalcY = MathUtil.clamp(pidCalcY, -1,1);
        pidCalcRot = MathUtil.clamp(pidCalcRot, -1,1);
        driveSubsystem.drive(-pidCalcX,pidCalcY, pidCalcRot);

        if (Constants.DebugInfo.debugAlign)
        {
            SmartDashboard.putNumber("tz", aprilDist);
            SmartDashboard.putNumber("tx", aprilX);
            SmartDashboard.putNumber("ry", aprilRot);
            SmartDashboard.putNumber("pidCalcX", pidCalcX);
            SmartDashboard.putNumber("pidCalcY", pidCalcY);
            SmartDashboard.putNumber("pidCalcRot", pidCalcRot);
        }
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
