package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomousCommands.AprilAlign;
import frc.robot.commands.autonomousCommands.AutoCoralIntake;
import frc.robot.commands.autonomousCommands.CoralDispCommand;
import frc.robot.commands.autonomousCommands.ToggleStop;
import frc.robot.commands.coralCommands.CoralHozPresetCommand;
import frc.robot.commands.liftCommands.LiftPresetCommand;
import frc.robot.control.CommandSwitchController;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightBackwar;
    private final LimelightSubsystem limelightForward;
    private final CoralSubsystem coralSubsystem;
    private final LiftSubsystem lift;
    private final AlgaeSubsystem algaeSubsystem;

    private final CommandSwitchController autoController;
    private final PIDController xController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController yController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1.0, 0.0, 0.0);

    private String source;
    private String reefFace;
    private AprilAlign.AprilPositions placerLoc;
    private LiftPresets liftLoc;
    private AlgaePresets algaeLoc;
    private AprilAlign.AprilPositions aprilOffset;

    public ChoreoSubsystem(RobotContainer robotContainer, CommandSwitchController autoController) {
        this.driveSubsystem = robotContainer.driveSubsystem;
        this.limelightBackwar = robotContainer.limelightBackwarSubsystem;
        this.limelightForward = robotContainer.limelightForwardSubsystem;
        this.coralSubsystem = robotContainer.coralSubsystem;
        this.algaeSubsystem = robotContainer.algaeSubsystem;
        this.lift = robotContainer.lift;

        this.autoController = autoController;
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

    public Command resetOdometry(String pathName) {
        return autoFactory.resetOdometry(pathName);
    }

    public Command selectTrajectory(String pathName) {
        return autoFactory.trajectoryCmd(pathName);
    }

//    public void scheduleAutoChooser(){
//        robotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
//    }


    public Command liftToPos(LiftPresets liftPreset) {
        return new LiftPresetCommand(lift, liftPreset);
    }

    public Command shootCoral() {
        return new CoralDispCommand(coralSubsystem, MotorDirection.REVERSE, 1);
    }

    public Command getCoral() {
        return new AutoCoralIntake(coralSubsystem);
    }

    public Command coralToPos(CoralPresets preset) {
        return new CoralHozPresetCommand(coralSubsystem, preset);
    }

    public Command goTo(String startLoc, String endLoc) {
        return new ParallelCommandGroup(
                new ToggleStop(driveSubsystem, false),
                new InstantCommand(() -> SmartDashboard.putString("TargetPath", startLoc + endLoc)),
                selectTrajectory(startLoc + endLoc),
                new ToggleStop(driveSubsystem, true));
    }

    public Command setRobotAt(String startLoc, String endLoc, LiftPresets liftLoc, CoralPresets coralLoc) {
        return new ParallelCommandGroup(
                liftToPos(liftLoc),
                coralToPos(coralLoc),
                goTo(startLoc, endLoc)
        );
    }

    public Command alignToApril(LimelightSubsystem limelightSubsystem, AprilAlign.AprilPositions aprilOffset) {
        return new SequentialCommandGroup(
                new AprilAlign(driveSubsystem, limelightSubsystem, .42, aprilOffset),
                new WaitCommand(.2)
        );
    }

    public Command goToAndPlace(String startLoc, String endLoc, LiftPresets liftLoc, CoralPresets coralLoc, AprilAlign.AprilPositions aprilOffset) {
        return new SequentialCommandGroup(
                setRobotAt(startLoc, endLoc, liftLoc, coralLoc),
                alignToApril(limelightForward, aprilOffset),
                resetOdometry(endLoc + "SourceN"), //TODO DO BETTER
                shootCoral());
    }

    public SequentialCommandGroup getBargeNAuto() {
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
//                choreo.resetOdometry("FieldRun"),
//                choreo.selectTrajectory("FieldRun"));
                resetOdometry("BargeNReefNE"),
                goToAndPlace("BargeN", "ReefNE", LiftPresets.BOTTOM_REEF, CoralPresets.CENTER_POS, AprilAlign.AprilPositions.CENTER),
                setRobotAt("ReefNE", "SourceN", LiftPresets.BOTTOM, CoralPresets.CENTER_POS));

    }

    public void updateTeleAuto() {
        updateSource();
        updateLift();
        updateAlgae();
        updatePlacer();
        updateReefFace();
    }

    public void updateSource() {
        String[] sourceLocs = {"SourceN", "SourceS"};
        for (int i = 1; i <= 2; i++) {
            if (autoController.button(i).getAsBoolean()) {
                this.source = sourceLocs[i];
            }
        }
    }

    public void updateReefFace() {
        String[] reefLocs = {"ReefW", "ReefNW", "ReefNE", "ReefE", "ReefSE", "ReefSW"};
        for (int i = 1; i <= 6; i++) {
            if (autoController.button(i + 2).getAsBoolean()) {
                this.reefFace = reefLocs[i-1];
            }
        }
    }

    public void updateLift() {
        LiftPresets[] liftPos = {LiftPresets.BOTTOM, LiftPresets.BOTTOM_REEF, LiftPresets.MIDDLE_REEF, LiftPresets.TOP_REEF, LiftPresets.ALGAE_LOW, LiftPresets.ALGAE_HIGH};
        for (int i = 1; i <= 6; i++) {
            if (autoController.button(i + 8).getAsBoolean()) {
                liftLoc = liftPos[i-1];
            }
        }
    }

    public void updatePlacer() {
        AprilAlign.AprilPositions[] aprilOffsets = {AprilAlign.AprilPositions.LEFT, AprilAlign.AprilPositions.CENTER, AprilAlign.AprilPositions.CENTER};
        for (int i = 1; i <= 3; i++) {
            if (autoController.button(i + 14).getAsBoolean()) {
                placerLoc = aprilOffsets[i-1];
            }
        }
    }

    public void updateAlgae() {
        AlgaePresets[] algaeLocs = {AlgaePresets.MINIMUM, AlgaePresets.MAXIMUM, AlgaePresets.GRAB};
        for (int i = 1; i <= 3; i++) {
            if (autoController.button(i + 20).getAsBoolean()){
                algaeLoc = algaeLocs[i-1];
            }
        }
    }


    public SequentialCommandGroup runTeleAuto() {
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                resetOdometry(source + reefFace),
                goToAndPlace(source, reefFace, liftLoc, CoralPresets.CENTER_POS, placerLoc),
                setRobotAt(reefFace, source, LiftPresets.BOTTOM, CoralPresets.CENTER_POS),
                alignToApril(limelightBackwar, AprilAlign.AprilPositions.CENTER));
    }

    @Override
    public void periodic() {
        updateTeleAuto();
        new InstantCommand(() -> {updateAlgae();updateTeleAuto();});
    }
}
