package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilAlign extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pidHoz = new PIDController(0.05,0,0);
    PIDController pidVert = new PIDController(0.05 * 60, 0, 0);
    PIDController pidRot = new PIDController(2,0,0);
    double aprilX;
    double aprilDist;
    double aprilRot; // Degrees
    double targetDist; // Meters
    double endTime;


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
    public void initialize(){
        endTime = Timer.getTimestamp() + Constants.Timeouts.aprilTimeout;
    }


    @Override
    public void execute() {
        aprilDist = limelightSubsystem.tz;
        aprilX = limelightSubsystem.tx;
        aprilRot = limelightSubsystem.ry;
        if(limelightSubsystem.id == -1)
        {
            return;
        }

        double pidCalcX = pidHoz.calculate(aprilX, 0);
        double pidCalcY = pidVert.calculate(aprilDist, targetDist);
        double pidCalcRot = pidRot.calculate(aprilRot, 0);

        pidCalcX = MathUtil.clamp(pidCalcX, -Constants.SpeedConstants.APRIL_ALIGN_SPEED, Constants.SpeedConstants.APRIL_ALIGN_SPEED);
        pidCalcY = MathUtil.clamp(pidCalcY, -Constants.SpeedConstants.APRIL_ALIGN_SPEED, Constants.SpeedConstants.APRIL_ALIGN_SPEED);
        pidCalcRot = MathUtil.clamp(pidCalcRot, -Constants.SpeedConstants.APRIL_ALIGN_SPEED, Constants.SpeedConstants.APRIL_ALIGN_SPEED);
        driveSubsystem.drive(-pidCalcX, -pidCalcY, pidCalcRot);

        if (Constants.DebugInfo.debugAlign)
        {
            SmartDashboard.putNumber("tz", aprilDist);
            SmartDashboard.putNumber("tzTarget", targetDist);
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
        return (Math.abs(aprilX) < .1 &&
                Math.abs(aprilRot) < 0.5 &&
                Math.abs(aprilDist - targetDist) < .1)
                || Timer.getTimestamp() >= endTime;
    }

}
