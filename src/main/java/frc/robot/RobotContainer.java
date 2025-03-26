package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.algaeCommands.AlgaePushCommand;
import frc.robot.commands.autonomousCommands.*;
import frc.robot.commands.coralCommands.CoralDispenserCommand;
import frc.robot.commands.coralCommands.CoralHozPidControl;
import frc.robot.commands.liftCommands.LiftPidControl;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.Algae.AlgaePresets;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Autonomous.ChoreoSubsystem;
import frc.robot.subsystems.Autonomous.RobotGoal;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Lift.LiftPresets;
import frc.robot.subsystems.Lift.LiftSubsystem;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Other.MotorDirection;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    private final CommandSwitchController customController = new CommandSwitchController(OperatorConstants.JOYSTICK_CONTROLLELR_PORT);

    public final DriveSubsystem driveSubsystem;
    public final LedSubsystem ledSubsystem = new LedSubsystem();
    public final LimelightSubsystem limelightForwardSubsystem = new LimelightSubsystem("forward", .42, false);
    public final LimelightSubsystem limelightBackwarSubsystem = new LimelightSubsystem("backwar", .60, true);

    public final LiftSubsystem lift;
    public final CoralSubsystem coralSubsystem;
    public final AlgaeSubsystem algaeSubsystem;
    private final ChoreoSubsystem choreo;
    private final DriveDefaultCommand driveDefaultCommand;
    public RobotGoal teleGoal = new RobotGoal();

    public RobotContainer() {

        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> -baseController.getLeftY(), () -> -(-baseController.getRightX()));
// THIS LINE ALLOWS D-PAD DRIVE BASE MOVEMENT driveDefaultCommand = new DriveDefaultCommand(() -> baseController.povRight().getAsBoolean() ? 0.5 :(baseController.povLeft().getAsBoolean() ? -0.5 : 0.),() -> baseController.povUp().getAsBoolean() ? 0.5 :(baseController.povDown().getAsBoolean() ? -0.5 : 0.),() -> 0);
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
        lift = new LiftSubsystem(driveSubsystem.chassis::SetAccelerationLimit, coralSubsystem, algaeSubsystem);
        lift.setDefaultCommand(new LiftPidControl(lift, () -> -payloadController.getLeftY(),
                payloadController.zl().and(payloadController.zr())));

        choreo = new ChoreoSubsystem(this);
        configureBindings();
    }

    private void configureBindings() {
        configBase();
        configPayload();
        configCustom();
    }

    private void configBase(){
        //TODO                             BASE CONTROLLER:    https://www.canva.com/design/DAGgzEn4UfA/D4Ydez6DajIAjL2_aeNujQ/edit

        // FIELD CENTRIC
        baseController.button(13).onTrue(new InstantCommand(() -> DriveDefaultCommand.isFieldCentric = !DriveDefaultCommand.isFieldCentric));

        // CORAL OUTPUT/INPUT
        baseController.r().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.FORWARD));
        baseController.l().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.REVERSE));

        // CORAL HORIZONTAL
        baseController.povRight().whileTrue(new CoralHozPidControl(coralSubsystem, () -> 0.5));
        baseController.povLeft().whileTrue(new CoralHozPidControl(coralSubsystem, () -> -0.5));

        // ALGAE OUTPUT/INPUT
        baseController.zr().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.FORWARD));
        baseController.zl().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.REVERSE));

        // ALGAE ANGLE
        baseController.x().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.FORWARD)));
        baseController.x().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));

        baseController.b().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.REVERSE)));
        baseController.b().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));

        // LIFT CONTROL
        baseController.povUp().whileTrue(new LiftPidControl(lift,() -> .5, () -> true));
        baseController.povDown().whileTrue(new LiftPidControl(lift,() -> -.5,  () -> true));

        // APRIL TAG ALIGN
        baseController.y().whileTrue(choreo.alignToApril(limelightForwardSubsystem, AprilAlign.AprilPositions.LEFT));
        baseController.a().whileTrue(choreo.runTeleAuto(new RobotGoal()
                .setStart("SourceN")
                .setEnd("ReefNW")
                .setLift(LiftPresets.ALGAE_LOW)
        ));

        baseController.button(14).whileTrue(choreo.alignToApril(limelightBackwarSubsystem, AprilAlign.AprilPositions.CENTER));

        // READY TO INTAKE
        //baseController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));
    }

    private void configPayload(){
        //TODO                  PAYLOAD CONTROLLER:    https://www.canva.com/design/DAGgzEn4UfA/D4Ydez6DajIAjL2_aeNujQ/edit

        payloadController.a().whileTrue(choreo.liftToPos(LiftPresets.BOTTOM));
        payloadController.x().whileTrue(choreo.liftToPos(LiftPresets.BOTTOM_REEF));
        payloadController.y().whileTrue(choreo.liftToPos(LiftPresets.MIDDLE_REEF));
        payloadController.b().whileTrue(choreo.liftToPos(LiftPresets.TOP_REEF));
        payloadController.povDown().whileTrue(choreo.liftToPos(LiftPresets.ALGAE_HIGH));
        payloadController.povUp().whileTrue(choreo.liftToPos(LiftPresets.ALGAE_LOW));
    }

    private void configCustom(){
        //TODO                  CUSTOM CONTROLLER
        customController.button(1).onTrue(new SequentialCommandGroup()); //To be edited, use it for whatever
        customController.button(2).onTrue(new InstantCommand());
        //Reset teleGoal
        customController.button(3).onTrue(new InstantCommand(() -> teleGoal = teleGoal.reset()));
        //AprilAlign
        customController.button(4).whileTrue(choreo.alignToApril(limelightForwardSubsystem, teleGoal));
        //GO TO, PLACE, RETURN
        customController.button(5).whileTrue(choreo.runTeleAuto(teleGoal));
        //Stop??
        customController.button(6).onTrue(new InstantCommand());

        //Choose Source Location
        customController.button(7).onTrue(new InstantCommand(() -> teleGoal.setStart("SourceN")));
        customController.button(8).onTrue(new InstantCommand(() -> teleGoal.setStart("SourceS")));

        //Choose Reef Location
        customController.button(9).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefW")));
        customController.button(10).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefNW")));
        customController.button(11).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefNE")));
        customController.button(12).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefE")));
        customController.button(13).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefSE")));
        customController.button(14).onTrue(new InstantCommand(() -> teleGoal.setEnd("ReefSW")));

        //Intake and shoot coral
        customController.button(15).whileTrue(choreo.getCoral());
        customController.button(16).whileTrue(choreo.shootCoral());

        //Place on reef location
        customController.button(17).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.BOTTOM_REEF)
                .setOffset(AprilAlign.AprilPositions.LEFT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));
        customController.button(18).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.MIDDLE_REEF)
                .setOffset(AprilAlign.AprilPositions.LEFT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));
        customController.button(19).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.TOP_REEF)
                .setOffset(AprilAlign.AprilPositions.LEFT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));
        customController.button(20).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.BOTTOM_REEF)
                .setOffset(AprilAlign.AprilPositions.RIGHT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));
        customController.button(21).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.MIDDLE_REEF)
                .setOffset(AprilAlign.AprilPositions.RIGHT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));
        customController.button(22).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.TOP_REEF)
                .setOffset(AprilAlign.AprilPositions.RIGHT)
                .setAlgae(AlgaePresets.SAFE_MOVE)));

        //Will go to net
        customController.button(23).onTrue(new InstantCommand());

        //Grab algae at
        customController.button(24).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.ALGAE_HIGH)
                .setAlgae(AlgaePresets.GRAB)
                .setOffset(AprilAlign.AprilPositions.CENTER)));
        customController.button(25).onTrue(new InstantCommand(() -> teleGoal.setLift(LiftPresets.ALGAE_LOW)
                .setAlgae(AlgaePresets.GRAB)
                .setOffset(AprilAlign.AprilPositions.CENTER)));

        //Push, intake, and push algae
        customController.button(26).whileTrue(new AlgaePushCommand(algaeSubsystem, MotorDirection.FORWARD));
        customController.button(27).whileTrue(new AlgaePushCommand(algaeSubsystem, MotorDirection.REVERSE));
        customController.button(28).whileTrue(new AlgaePushCommand(algaeSubsystem, MotorDirection.FORWARD));

        customController.button(29).onTrue(new InstantCommand()); //Will be processor
        //customController.button(30).onTrue(teleGoal.setLift(LiftPresets.Trough)) No Trough Enum made, nor intended to use
        customController.button(31).onTrue(new InstantCommand()); //Unsure of use

        customController.button(32).onTrue(new InstantCommand()); //Speed 1
        customController.button(33).onTrue(new InstantCommand()); //Speed 2
        customController.button(34).onTrue(new InstantCommand()); //Speed 3
        customController.button(35).onTrue(new InstantCommand()); //Speed 4
        customController.button(36).onTrue(new InstantCommand()); //Speed 5
        customController.button(37).onTrue(new InstantCommand()); //Climb Speed 1
        customController.button(38).onTrue(new InstantCommand()); //Climb Speed 2
        customController.button(39).onTrue(new InstantCommand()); //Climb Speed 3

        customController.button(40).onTrue(new InstantCommand()); //Go to Cage 1
        customController.button(41).onTrue(new InstantCommand()); //Go to Cage 2
        customController.button(42).onTrue(new InstantCommand()); //Go to Cage 3

        customController.button(43).onTrue(new InstantCommand()); //Deploy climber
        customController.button(44).onTrue(new InstantCommand()); //Climb up
        customController.button(45).onTrue(new InstantCommand()); //Climb down
    }


    public Command getAutonomousCommand() {
        return choreo.getBargeNAuto();

    }
}
