package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Autonomous.ChoreoSubsystem;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import jdk.dynalink.Operation;

import java.util.HashMap;
import java.util.function.Supplier;

public class AprilAlign extends Command {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pidHoz = new PIDController(0.1 * .6,0,0);
    PIDController pidVert = new PIDController(0.1 * 140, 0, 0);
    PIDController pidRot = new PIDController(2,0,0);
    double aprilX;
    double aprilDist;
    double aprilRot; // Degrees
    double targetDist; // Meters
    double endTime;
    double aprilOffset;
    int successes;
    ChoreoSubsystem choreo;
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

    public int ignore(){
        return 1;
    }

    public AprilAlign(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double targetDist, AprilPositions aprilOff, ChoreoSubsystem choreo){
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
        successes = 0;
    }


    @Override
    public void execute() {
        aprilDist = limelightSubsystem.tz;
        aprilX = limelightSubsystem.tx;
        aprilRot = limelightSubsystem.ry;

        if(limelightSubsystem.isReversed)
        {
            aprilX = -aprilX;
        }
        if(limelightSubsystem.id == -1)
        {
            return;
        }

        double pidCalcX = pidHoz.calculate(aprilX, aprilOffset);
        double pidCalcY = pidVert.calculate(aprilDist, targetDist);
        double pidCalcRot = pidRot.calculate(aprilRot, 0);

        if(limelightSubsystem.isReversed)
        {
            pidCalcY = - pidCalcY;
        }

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

    public static class AprilDict{
        HashMap<Integer, String> aprilIdsLoc = new HashMap<>();
        public AprilDict(){
            aprilIdsLoc.put(1,"SourceN");
            aprilIdsLoc.put(2,"SourceS");
            aprilIdsLoc.put(3,"");
            aprilIdsLoc.put(4,"");
            aprilIdsLoc.put(5,"");
            aprilIdsLoc.put(6,"ReefNW");
            aprilIdsLoc.put(7,"ReefW");
            aprilIdsLoc.put(8,"ReefSW");
            aprilIdsLoc.put(9,"ReefSE");
            aprilIdsLoc.put(10,"ReefE");
            aprilIdsLoc.put(11,"ReefNE");
            aprilIdsLoc.put(12,"SourceS");
            aprilIdsLoc.put(13,"SourceN");
            aprilIdsLoc.put(14,"");
            aprilIdsLoc.put(15,"");
            aprilIdsLoc.put(16,"");
            aprilIdsLoc.put(17,"ReefSW");
            aprilIdsLoc.put(18,"ReefW");
            aprilIdsLoc.put(19,"ReefNW");
            aprilIdsLoc.put(20,"ReefNE");
            aprilIdsLoc.put(21,"ReefE");
            aprilIdsLoc.put(22,"ReefSE");
        }

        public String getLocation(int ID){
            return aprilIdsLoc.getOrDefault(ID, "");
        }
    }

    public void resetBasedOnLoc(int ID){
        AprilDict aprilDict = new AprilDict();
        String loc = aprilDict.getLocation(ID);
        if (!loc.isEmpty()){
            if (loc.contains("Reef")) {
                choreo.resetOdometry(loc, "SourceN");
            } else {
                choreo.resetOdometry(loc, "ReefN");
            }
        }
    }

    @Override
    public boolean isFinished() {
        if ((Math.abs(aprilX + aprilOffset) < .1 &&
                Math.abs(aprilRot) < 0.5 &&
                Math.abs(aprilDist - targetDist) < .1)
                || Timer.getTimestamp() >= endTime)
        {
            successes++;
            if(successes > 50) {
                //resetBasedOnLoc((int) limelightSubsystem.id);
                return true;
            }
        }
        return false;
    }

}
