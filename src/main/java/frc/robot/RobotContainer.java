package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.algaeCommands.AlgaePushCommand;
import frc.robot.commands.autonomousCommands.AprilAlign;
import frc.robot.commands.autonomousCommands.AutoCoralDispCommand;
import frc.robot.commands.autonomousCommands.AutoCoralIntake;
import frc.robot.commands.autonomousCommands.ToggleStop;
import frc.robot.commands.coralCommands.CoralDispenserCommand;
import frc.robot.commands.coralCommands.CoralHozPidControl;
import frc.robot.commands.liftCommands.LiftPidControl;
import frc.robot.commands.liftCommands.LiftPresetCommand;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);

    public final DriveSubsystem driveSubsystem;
    public final LimelightSubsystem limelightForwardSubsystem = new LimelightSubsystem("forward");
    public final LimelightSubsystem limelightBackwarSubsystem = new LimelightSubsystem("backwar");

    public final LiftSubsystem lift;
    private final DriveDefaultCommand driveDefaultCommand;
    CoralSubsystem coralSubsystem;
    AlgaeSubsystem algaeSubsystem;
    ChoreoSubsystem choreo;

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> -baseController.getLeftY(), () -> -(-baseController.getRightX()));
// THIS LINE ALLOWS D-PAD DRIVE BASE MOVEMENT driveDefaultCommand = new DriveDefaultCommand(() -> baseController.povRight().getAsBoolean() ? 0.5 :(baseController.povLeft().getAsBoolean() ? -0.5 : 0.),() -> baseController.povUp().getAsBoolean() ? 0.5 :(baseController.povDown().getAsBoolean() ? -0.5 : 0.),() -> 0);
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        choreo = new ChoreoSubsystem(driveSubsystem);
        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
        lift = new LiftSubsystem(driveSubsystem.chassis::SetAccelerationLimit, coralSubsystem, algaeSubsystem);
        lift.setDefaultCommand(new LiftPidControl(lift, () -> -payloadController.getLeftY(),
                payloadController.zl().and(payloadController.zr())));

        choreo.scheduleAutoChooser();
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
        baseController.povLeft().whileTrue(new CoralHozPidControl(coralSubsystem, () -> 0.5));
        baseController.povRight().whileTrue(new CoralHozPidControl(coralSubsystem, () -> -0.5));

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
        baseController.y().whileTrue(new AutoCoralIntake(coralSubsystem));

        // READY TO INTAKE
        //baseController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));

        /* TO ADD
        Left "Start": Reset Gyro
        Minus: Stop going to Pos
        Plus: Go to next Pos
        */

        //TODO                  PAYLOAD CONTROLLER:    https://www.canva.com/design/DAGgzEn4UfA/D4Ydez6DajIAjL2_aeNujQ/edit

        //payloadController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.STARTING));
        //payloadController.x().whileTrue(new LiftPresetCommand(lift, LiftPresets.TROUGH));
        //payloadController.y().whileTrue(new LiftPresetCommand(lift, LiftPresets.HEIGHT_1));
        //payloadController.b().whileTrue(new LiftPresetCommand(lift, LiftPresets.HEIGHT_2));

        /*payloadController.povRight().whileTrue( new AlgaeAnglePresetCommand(algaeSubsystem, AlgaePresets.DEFAULT_DOWN));
        payloadController.povDown().whileTrue(  new AlgaeAnglePresetCommand(algaeSubsystem, AlgaePresets.POSITION_2));
        payloadController.povUpLeft().whileTrue(new AlgaeAnglePresetCommand(algaeSubsystem, AlgaePresets.POSITION_3));
        payloadController.povUp().whileTrue(    new AlgaeAnglePresetCommand(algaeSubsystem, AlgaePresets.POSITION_4)); */


    }
    public Command followAutoPath(String pathName){
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                choreo.SelectTrajectory(pathName),
                new WaitCommand(.5)
        );
    }

    public Command alignToApril(LimelightSubsystem limelightSubsystem){
        return new SequentialCommandGroup(
                new AprilAlign(driveSubsystem,limelightSubsystem,.3),
                new WaitCommand(1)
        );
    }

    public Command liftToPos(LiftPresets liftPreset){
        return new SequentialCommandGroup(
                new LiftPresetCommand(lift, liftPreset),
                new WaitCommand(1)
        );
    }

    public Command shootCoral(){
        return new SequentialCommandGroup(
                new AutoCoralDispCommand(coralSubsystem, MotorDirection.FORWARD, 1),
                new WaitCommand(1)
        );
    }
    public Command getCoral(){
        return new SequentialCommandGroup(
                new AutoCoralIntake(coralSubsystem),
                new WaitCommand(1)
        );
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("MoveTurnMove"),
                choreo.SelectTrajectory("MoveTurnMove"),
                //followAutoPath("moveToReef"),
                //alignToApril(limelightForwardSubsystem),
                //liftToPos(LiftPresets.TROUGH),
                //shootCoral(),
                //liftToPos(LiftPresets.STARTING),
                //followAutoPath("moveToSource"),
                //getCoral(),
                new ToggleStop(driveSubsystem, false));
    }
}
