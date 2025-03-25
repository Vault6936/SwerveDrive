package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class MoveToPosCommand extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pidStrafe = new PIDController(0.5,0,0);
    PIDController pidRot = new PIDController(0.5,0,0);
    double currX; // Meters
    double currY; // Meters
    double currRot; /* IN RADIANS */
    double targetX; // Meters
    double targetY; // Meters
    double targetRot; /* IN RADIANS */
    double endTime;

    public MoveToPosCommand(DriveSubsystem driveSubsystem, double x, double y, double rot /* IN RADIANS */){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        targetX = x;
        targetY = y;
        targetRot = rot;

        currX = driveSubsystem.currentPose.getX();
        currY = driveSubsystem.currentPose.getY();
        currRot = driveSubsystem.currentPose.getRotation().getRadians();
    }

    public MoveToPosCommand(DriveSubsystem driveSubsystem, Pose2d targetPose)
    {
        this(driveSubsystem, targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians());
    }

    @Override
    public void initialize(){
        endTime = Timer.getTimestamp() + Constants.Timeouts.moveToPosTimeout;
    }

    @Override
    public void execute() {
        currX = driveSubsystem.currentPose.getX();
        currY = driveSubsystem.currentPose.getY();
        currRot = driveSubsystem.currentPose.getRotation().getRadians();

        double pidCalcX = pidStrafe.calculate(currX, targetX);
        double pidCalcY = pidStrafe.calculate(currY, targetY);
        double pidCalcRot = pidRot.calculate(currRot, targetRot);

        pidCalcX = MathUtil.clamp(pidCalcX,-1,1);
        pidCalcY = MathUtil.clamp(pidCalcY,-1,1);
        pidCalcRot = MathUtil.clamp(pidCalcRot,-1,1);

        driveSubsystem.drive(-pidCalcX,pidCalcY, pidCalcRot,true);
    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }

    @Override
    public boolean isFinished()
    {
        return  (Math.abs(currX - targetX) < .1 &&
                Math.abs(currY - targetY) < .1 &&
                Math.abs(currRot - targetRot) < .1)
                || Timer.getTimestamp() > endTime;
    }

}
