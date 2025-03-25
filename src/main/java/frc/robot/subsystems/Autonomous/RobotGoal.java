package frc.robot.subsystems.Autonomous;

import frc.robot.commands.autonomousCommands.AprilAlign;
import frc.robot.subsystems.Algae.AlgaePresets;
import frc.robot.subsystems.Coral.CoralPresets;
import frc.robot.subsystems.Lift.LiftPresets;

public class RobotGoal {
    private String start;
    private String end;
    private LiftPresets liftGoal;
    private AlgaePresets algaeGoal;
    private CoralPresets coralGoal;
    private AprilAlign.AprilPositions aprilOffset;

    public RobotGoal setStart(String startLoc){
        start = startLoc;
        return this;
    }

    public RobotGoal setEnd(String endLoc){
        end = endLoc;
        return this;
    }

    public RobotGoal setLift(LiftPresets liftGoal){
        this.liftGoal = liftGoal;
        return this;
    }

    public RobotGoal setAlgae(AlgaePresets algaeGoal){
        this.algaeGoal = algaeGoal;
        return this;
    }

    public RobotGoal setCoral(CoralPresets coralGoal){
        this.coralGoal = coralGoal;
        return this;
    }

    public RobotGoal setOffset(AprilAlign.AprilPositions aprilOffset){
        this.aprilOffset = aprilOffset;
        return this;
    }
    public String getEnd(){
        if (!checkNull(end))
            return end;
        else return "";
    }

    public String getStart(){
        if (!checkNull(start))
            return start;
        else return "";
    }

    public String getPathname(){
        if (!checkNull(start + end))
            return start + end;
        else return "";
    }

    public LiftPresets getLift(){
        if (!checkNull(liftGoal))
            return liftGoal;
        else return LiftPresets.SAFE_LOW_DRIVE;
    }

    public AlgaePresets getAlgae(){
        if (!checkNull(algaeGoal))
            return algaeGoal;
        else return AlgaePresets.SAVE_MOVE;
    }

    public CoralPresets getCoral(){
        if (!checkNull(coralGoal))
            return coralGoal;
        else return CoralPresets.CENTER_POS;
    }
    public AprilAlign.AprilPositions getOffset(){
        if (!checkNull(aprilOffset))
            return aprilOffset;
        else return AprilAlign.AprilPositions.CENTER;
    }

    public boolean checkNull(Object param){
        return param == null;
    }

    public RobotGoal reset(){
        return new RobotGoal();
    }

    public RobotGoal copy(){
        return new RobotGoal()
                .setStart(start)
                .setEnd(end)
                .setOffset(aprilOffset)
                .setCoral(coralGoal)
                .setLift(liftGoal)
                .setAlgae(algaeGoal);
    }
}
