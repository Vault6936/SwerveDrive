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
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Autonomous.ChoreoSubsystem;
import frc.robot.subsystems.Autonomous.RobotGoal;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Lift.LiftPresets;
import frc.robot.subsystems.Lift.LiftSubsystem;
import frc.robot.subsystems.Other.LimelightSubsystem;
import frc.robot.subsystems.Other.MotorDirection;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    private final CommandSwitchController customController = new CommandSwitchController(OperatorConstants.JOYSTICK_CONTROLLELR_PORT);

    public final DriveSubsystem driveSubsystem;
    public final LimelightSubsystem limelightForwardSubsystem = new LimelightSubsystem("forward");
    public final LimelightSubsystem limelightBackwarSubsystem = new LimelightSubsystem("backwar");

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
        baseController.a().whileTrue(choreo.alignToApril(limelightForwardSubsystem, AprilAlign.AprilPositions.CENTER));

        // READY TO INTAKE
        //baseController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));


        //TODO                  PAYLOAD CONTROLLER:    https://www.canva.com/design/DAGgzEn4UfA/D4Ydez6DajIAjL2_aeNujQ/edit

        payloadController.a().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.BOTTOM)));
        payloadController.x().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.BOTTOM_REEF)));
        payloadController.y().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.MIDDLE_REEF)));
        payloadController.b().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.TOP_REEF)));
        payloadController.povDown().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.ALGAE_HIGH)));
        payloadController.povUp().whileTrue(choreo.liftToPos(teleGoal.reset().setLift(LiftPresets.ALGAE_LOW)));


        //TODO                  CUSTOM CONTROLLER
        customController.button(1).onTrue(new SequentialCommandGroup()); //To be edited, use it for whatever
        customController.button(2).onTrue(new InstantCommand());
        customController.button(3).onTrue(new InstantCommand(() -> teleGoal = teleGoal.reset()));       //Reset pose
        customController.button(4).whileTrue(choreo.alignToApril(limelightForwardSubsystem, teleGoal)); //April align
        customController.button(5).whileTrue(choreo.runTeleAuto(teleGoal));                             //GO TO, PLACE, RETURN
        customController.button(6).onTrue(new InstantCommand());

        customController.button(7).onTrue(new InstantCommand(() -> teleGoal.setStart("SourceN")));
        customController.button(8).onTrue(new InstantCommand(() -> teleGoal.setStart("SourceS")));

    }


    public Command getAutonomousCommand() {
        return choreo.getBargeNAuto();

    }
}
