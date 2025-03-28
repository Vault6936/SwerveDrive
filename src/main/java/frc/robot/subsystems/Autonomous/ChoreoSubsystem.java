package frc.robot.subsystems.Autonomous;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.algaeCommands.AlgaeAnglePresetCommand;
import frc.robot.commands.autonomousCommands.AprilAlign;
import frc.robot.commands.autonomousCommands.AutoCoralIntake;
import frc.robot.commands.autonomousCommands.CoralDispCommand;
import frc.robot.commands.autonomousCommands.ToggleStop;
import frc.robot.commands.coralCommands.CoralHozPresetCommand;
import frc.robot.commands.liftCommands.LiftPresetCommand;
import frc.robot.subsystems.Algae.AlgaePresets;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralPresets;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Lift.LiftPresets;
import frc.robot.subsystems.Lift.LiftSubsystem;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

import java.util.function.Supplier;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightBackwar;
    private final LimelightSubsystem limelightForward;
    private final CoralSubsystem coralSubsystem;
    private final LiftSubsystem lift;
    private final AlgaeSubsystem algaeSubsystem;
    private final RobotContainer robotContainer;

    private final PIDController xController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController yController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1.0, 0.0, 0.0);

    public ChoreoSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.driveSubsystem = robotContainer.driveSubsystem;
        this.limelightBackwar = robotContainer.limelightBackwarSubsystem;
        this.limelightForward = robotContainer.limelightForwardSubsystem;
        this.coralSubsystem = robotContainer.coralSubsystem;
        this.algaeSubsystem = robotContainer.algaeSubsystem;
        this.lift = robotContainer.lift;

        autoFactory = new AutoFactory(() -> driveSubsystem.currentPose, driveSubsystem::poseReset, this::followTrajectory,
                false, driveSubsystem);
        xController.setIntegratorRange(-50, 50);
        yController.setIntegratorRange(-50, 50);
        headingController.setIntegratorRange(-6, 6);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void followTrajectory(SwerveSample sample) {
        double forwardCalc = -xController.calculate(driveSubsystem.currentPose.getX(), sample.x);
        double leftCalc = -yController.calculate(driveSubsystem.currentPose.getY(), sample.y);
        double headingCalc = headingController.calculate(driveSubsystem.currentPose.getRotation().getRadians(), sample.heading);

//        ChassisSpeeds speeds = new ChassisSpeeds(
//                -sample.vy, //+ forwardCalc,
//                sample.vx, // + leftCalc,
//                0 /*sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading*/
//        );

        if (Constants.DebugInfo.debugChoreo) {
//            SmartDashboard.putNumber("ForwardCalc", forwardCalc);
//            SmartDashboard.putNumber("LeftCalc", leftCalc);
//            SmartDashboard.putNumber("LeftDiff", -sample.vy);
//            SmartDashboard.putNumber("ForwardDiff", sample.vy);
            SmartDashboard.putNumber("SampleHeading", Math.toDegrees(sample.heading));
            SmartDashboard.putNumber("SampleCalc", headingCalc);
            SmartDashboard.putNumber("SampleDiff", Math.toDegrees(sample.heading - driveSubsystem.currentPose.getRotation().getRadians()));
        }
        driveSubsystem.drive
                (MathUtil.clamp(-sample.vy + leftCalc, -1, 1),
                        MathUtil.clamp(sample.vx - forwardCalc, -1, 1),
                        MathUtil.clamp(-sample.omega - headingCalc, -0.5, 0.5),
                        true);
    }

    public Command resetOdometry(String start, String end) {
        if (checkPath(start, end)){
            System.out.println("Cannot reset odometry when one of the targets is blank");
            return new InstantCommand();
        } else {
            return autoFactory.resetOdometry(start + end);
        }
    }

    public Command selectTrajectory(String start, String end) {
        if (checkPath(start, end)){
            System.out.println("Cannot select a trajectory when one of the targets is blank");
            return new InstantCommand();
        } else {
            return autoFactory.trajectoryCmd(start + end);
        }
    }

    public boolean checkPath(String str1, String str2){
        return str1.isEmpty() || str2.isEmpty();
    }

    // CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL
    public Command shootCoral() {
        return new CoralDispCommand(coralSubsystem, MotorDirection.REVERSE, 1);
    }

    public Command getCoral() {
        return new AutoCoralIntake(coralSubsystem);
    }

    public Command coralToPos(RobotGoal robotGoal) {
        return coralToPos(robotGoal.getCoral());
    }

    public Command coralToPos(CoralPresets coralPos) {
        return new CoralHozPresetCommand(coralSubsystem, coralPos);
    }

    // LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT
    public Command liftToPos(RobotGoal robotGoal) {
        return liftToPos(robotGoal.getLift());
    }

    public Command liftToPos(LiftPresets liftPos){
        return new LiftPresetCommand(lift, liftPos);
    }

    // ALGAE //// ALGAE //// ALGAE //// ALGAE //// ALGAE //// ALGAE //// ALGAE //// ALGAE
    public Command algaeToPos(RobotGoal robotGoal){
        return algaeToPos(robotGoal.getAlgae());
    }
    public Command algaeToPos(AlgaePresets algaePre){
        return new AlgaeAnglePresetCommand(algaeSubsystem, algaePre);
    }


    // MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT
    public Command goTo(RobotGoal robotGoal) {
        return new ParallelCommandGroup(
                new ToggleStop(driveSubsystem, false),
                selectTrajectory(robotGoal.getStart(), robotGoal.getEnd())
        );
    }

    public Command alignToApril(LimelightSubsystem limelight, AprilAlign.AprilPositions aprilOffset) {
        return new SequentialCommandGroup(
                new AprilAlign(driveSubsystem, limelight, limelight.flushOffset, aprilOffset, this),
                new WaitCommand(.2)
        );
    }

    public Command setRobotAt(RobotGoal robotGoal){
        Command needsLowerBef = new InstantCommand();

        SmartDashboard.putString("RobotGoal", robotGoal.toString());

        if (lift.getCurrentPosition() > LiftPresets.MIDDLE_REEF.position) {
            needsLowerBef = liftToPos(LiftPresets.MIDDLE_REEF);
        }

        Command needsHigherAft = new InstantCommand();
        if (robotGoal.getLift() == LiftPresets.TOP_REEF){
            robotGoal.setLift(LiftPresets.MIDDLE_REEF);
            needsHigherAft = new LiftPresetCommand(lift, LiftPresets.TOP_REEF);
        }
        SmartDashboard.putString("RobotGoalPost", robotGoal.toString());
        return new SequentialCommandGroup(
                needsLowerBef,
                new ParallelCommandGroup(
                    new InstantCommand(() -> SmartDashboard.putString("TargetPath", robotGoal.getPathname())),
                    goTo(robotGoal),
                    liftToPos(robotGoal),
                    coralToPos(robotGoal)),
                needsHigherAft);
    }

    public Command alignToApril(LimelightSubsystem limelight, RobotGoal robotGoal) {
        return alignToApril(limelight, robotGoal.getOffset());
    }

    public Command manualPlace(LiftPresets liftPre, CoralPresets coralPre, AprilAlign.AprilPositions aprilOffset){
        return new ParallelCommandGroup(
                liftToPos(liftPre),
                coralToPos(coralPre),
                alignToApril(limelightForward, aprilOffset)
        );
    }

    //AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS//
    public SequentialCommandGroup runAuto(String startLoc, String endLoc, String source){
        RobotGoal placeGoal = new RobotGoal().setStart(startLoc).setEnd(endLoc)
                .setLift(LiftPresets.BOTTOM_REEF)
                .setCoral(CoralPresets.CENTER_POS)
                .setOffset(AprilAlign.AprilPositions.RIGHT);

        // Defaults to a "return" position. Lift down, Coral Center, April Center
        RobotGoal sourceGoal = new RobotGoal().setStart(endLoc).setEnd(source);

        return goPlaceReturn(placeGoal, sourceGoal);
    }

    public SequentialCommandGroup goPlaceReturn(RobotGoal placeGoal, RobotGoal returnGoal){
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                resetOdometry(placeGoal.getStart(), placeGoal.getEnd()),
                setRobotAt(placeGoal),
                alignToApril(limelightForward, placeGoal),
                shootCoral(),
                resetOdometry(returnGoal.getStart(), returnGoal.getEnd()),
                alignToApril(limelightForward, returnGoal),
                setRobotAt(returnGoal),
                alignToApril(limelightBackwar, returnGoal));
    }

    public SequentialCommandGroup getBargeNReefNEAuto() {
        return runAuto("BargeN", "ReefNE", "SourceN");
    }

    public SequentialCommandGroup getBargeNReefNWAuto(){
        return runAuto("BargeN", "ReefNW", "SourceN");
    }

    public SequentialCommandGroup getBargeMReefNWAuto(){
        return runAuto("BargeM", "ReefNW", "SourceN");
    }

    public SequentialCommandGroup getBargeMReefSWAuto(){
        return runAuto("BargeM", "ReefSW", "SourceS");
    }

    public SequentialCommandGroup getBargeSReefSEAuto() {
        return runAuto("BargeS", "ReefSE", "SourceS");
    }

    public SequentialCommandGroup getBargeSReefSWAuto(){
        return runAuto("BargeS", "ReefSW", "SourceS");
    }

    //TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO//
    public SequentialCommandGroup runTeleAuto(RobotGoal teleGoal) {
        // Defaults to a "return" position. Lift down, Coral Center, April Center
        RobotGoal returnGoal = new RobotGoal().setStart(teleGoal.getEnd()).setEnd(teleGoal.getStart());
        return goPlaceReturn(teleGoal, returnGoal);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("TeleGoal", robotContainer.teleGoal.toString());
    }
}
