package frc.robot.subsystems.Autonomous;

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
import frc.robot.subsystems.Algae.AlgaePresets;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralPresets;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Lift.LiftPresets;
import frc.robot.subsystems.Lift.LiftSubsystem;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightBackwar;
    private final LimelightSubsystem limelightForward;
    private final CoralSubsystem coralSubsystem;
    private final LiftSubsystem lift;
    private final AlgaeSubsystem algaeSubsystem;

    private final PIDController xController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController yController = new PIDController(1.0, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1.0, 0.0, 0.0);

    public ChoreoSubsystem(RobotContainer robotContainer) {
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

    public Command resetOdometry(String pathName) {
        return autoFactory.resetOdometry(pathName);
    }

    public Command selectTrajectory(String pathName) {
        return autoFactory.trajectoryCmd(pathName);
    }

//    public void scheduleAutoChooser(){
//        robotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
//    }


    // CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL //// CORAL
    public Command shootCoral() {
        return new CoralDispCommand(coralSubsystem, MotorDirection.REVERSE, 1);
    }

    public Command getCoral() {
        return new AutoCoralIntake(coralSubsystem);
    }

    public Command coralToPos(RobotGoal robotGoal) {
        return new CoralHozPresetCommand(coralSubsystem, robotGoal.getCoral());
    }

    // LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT //// LIFT
    public Command liftToPos(RobotGoal robotGoal) {
        return new LiftPresetCommand(lift, robotGoal.getLift());
    }

    // MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT //// MOVEMENT
    public Command goTo(RobotGoal robotGoal) {
        return new ParallelCommandGroup(
                new ToggleStop(driveSubsystem, false),
                new InstantCommand(() -> SmartDashboard.putString("TargetPath", robotGoal.getPathname())),
                selectTrajectory(robotGoal.getPathname())
        );
                //new ToggleStop(driveSubsystem, true)); TODO CONFIRM IF THIS IS WANTED BEHAVIOR
    }

    public Command goBack(RobotGoal robotGoal) {
        String start = robotGoal.getStart();
        String end = robotGoal.getEnd();
        return goTo(robotGoal.copy().setEnd(start).setStart(end));
    }

    public Command setRobotAt(RobotGoal robotGoal) {
        return new ParallelCommandGroup(
                liftToPos(robotGoal),
                coralToPos(robotGoal),
                goTo(robotGoal)
        );
    }

    public Command alignToApril(LimelightSubsystem limelight, AprilAlign.AprilPositions aprilOffset) {
        return new SequentialCommandGroup(
                new AprilAlign(driveSubsystem, limelight, .42, aprilOffset),
                new WaitCommand(.2)
        );
    }

    public Command alignToApril(LimelightSubsystem limelight, RobotGoal robotGoal) {
        return alignToApril(limelight, robotGoal.getOffset());
    }

    public Command goToAndPlace(RobotGoal robotGoal) {
        return new SequentialCommandGroup(
                setRobotAt(robotGoal),
                alignToApril(limelightForward, robotGoal),
                resetOdometry(robotGoal.getEnd() + "SourceN"), //TODO DO BETTER
                shootCoral()
        );
    }

    //AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS////AUTONOMOUS//

    public SequentialCommandGroup getBargeNAuto() {
        RobotGoal robotGoal = new RobotGoal().setStart("BargeN").setEnd("ReefNE")
                .setLift(LiftPresets.BOTTOM_REEF)
                .setCoral(CoralPresets.CENTER_POS)
                .setOffset(AprilAlign.AprilPositions.CENTER);

        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                resetOdometry(robotGoal.getPathname()),
                goToAndPlace(robotGoal),
                setRobotAt(robotGoal.setLift(LiftPresets.BOTTOM).setCoral(CoralPresets.CENTER_POS)));

    }

    //TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO////TELEAUTO//

    public SequentialCommandGroup runTeleAuto(RobotGoal teleGoal) {
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                resetOdometry(teleGoal.getPathname()),
                goToAndPlace(teleGoal),
                setRobotAt(teleGoal)
                );
    }

    @Override
    public void periodic() {
    }
}
