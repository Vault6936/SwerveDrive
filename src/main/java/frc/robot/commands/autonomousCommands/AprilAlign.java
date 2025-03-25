package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class AprilAlign extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pidHoz = new PIDController(0.1 * 1.2,0,0);
    PIDController pidVert = new PIDController(0.1 * 140, 0, 0);
    PIDController pidRot = new PIDController(2,0,0);
    double aprilX;
    double aprilDist;
    double aprilRot; // Degrees
    double targetDist; // Meters
    double endTime;
    double aprilOffset;
    int sucesses;

    public enum AprilPositions
    {
        LEFT(22.5),
        CENTER(0),
        RIGHT(-22.5);
        public final double position;
        AprilPositions(double pos)
        {
            this.position = pos;
        }
    }

    public AprilAlign(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double targetDist, AprilPositions aprilOff){
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem);
        aprilX = limelightSubsystem.tx;
        aprilDist = limelightSubsystem.tz;
        aprilRot = limelightSubsystem.ry;
        this.targetDist = targetDist;
        aprilOffset = aprilOff.position;
    }

    @Override
    public void initialize(){
        endTime = Timer.getTimestamp() + Constants.Timeouts.aprilTimeout;
        sucesses = 0;
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

        double pidCalcX = pidHoz.calculate(aprilX, aprilOffset);
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
    public boolean isFinished() {
        if ((Math.abs(aprilX + aprilOffset) < .1 &&
                Math.abs(aprilRot) < 0.5 &&
                Math.abs(aprilDist - targetDist) < .1)
                || Timer.getTimestamp() >= endTime)
        {
            sucesses ++;
            if(sucesses > 50)
                return true;
        }
        return false;
    }

}
