package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.algaeCommands.AlgaePushCommand;
import frc.robot.commands.autonomousCommands.*;
import frc.robot.commands.coralCommands.CoralDispenserCommand;
import frc.robot.commands.coralCommands.CoralHozPidControl;
import frc.robot.commands.coralCommands.CoralHozPresetCommand;
import frc.robot.commands.liftCommands.LiftPidControl;
import frc.robot.commands.liftCommands.LiftPresetCommand;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    private final CommandSwitchController joystickController = new CommandSwitchController(OperatorConstants.JOYSTICK_CONTROLLELR_PORT);

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
        baseController.povUp().whileTrue(new LiftPidControl(lift,() -> .5, () -> false));
        baseController.povDown().whileTrue(new LiftPidControl(lift,() -> -.5,  () -> false));

        // APRIL TAG ALIGN
        baseController.y().whileTrue(alignToApril(limelightForwardSubsystem));

        // READY TO INTAKE
        //baseController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));

        /* TO ADD
        Left "Start": Reset Gyro
        Minus: Stop going to Pos
        Plus: Go to next Pos
        */

        //TODO                  PAYLOAD CONTROLLER:    https://www.canva.com/design/DAGgzEn4UfA/D4Ydez6DajIAjL2_aeNujQ/edit
        joystickController.button(1).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefNW"), goTo("SourceN", "ReefNW"), goTo("ReefNW", "SourceN")));
        joystickController.button(2).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefNE"), goTo("SourceN", "ReefNE"), goTo("ReefNE", "SourceN")));
        joystickController.button(3).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefE"), goTo("SourceN", "ReefE"), goTo("ReefE", "SourceN")));
        joystickController.button(4).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefSE"), goTo("SourceN", "ReefSE"), goTo("ReefSE", "SourceN")));
        joystickController.button(5).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefSW"), goTo("SourceN", "ReefSW"), goTo("ReefSW", "SourceN")));
        joystickController.button(6).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceNReefW"), goTo("SourceN", "ReefW"), goTo("ReefW", "SourceN")));

        joystickController.button(7).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefNW"), goTo("SourceS", "ReefNW"), goTo("ReefNW", "SourceS")));
        joystickController.button(8).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefNE"), goTo("SourceS", "ReefNE"), goTo("ReefNE", "SourceS")));
        joystickController.button(9).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefE"), goTo("SourceS", "ReefE"), goTo("ReefE", "SourceS")));
        joystickController.button(10).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefSE"), goTo("SourceS", "ReefSE"), goTo("ReefSE", "SourceS")));
        joystickController.button(11).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefSW"), goTo("SourceS", "ReefSW"), goTo("ReefSW", "SourceS")));
        joystickController.button(12).whileTrue(new SequentialCommandGroup( new ToggleStop(driveSubsystem, false),
                choreo.resetOdometry("SourceSReefW"), goTo("SourceS", "ReefW"), goTo("ReefW", "SourceS")));
    }

    public Command followPath(String pathName){
        return new SequentialCommandGroup(
                choreo.selectTrajectory(pathName),
                new WaitCommand(.2)
        );
    }

    public Command alignToApril(LimelightSubsystem limelightSubsystem){
        return new SequentialCommandGroup(
                new AprilAlign(driveSubsystem,limelightSubsystem,.54),
                new WaitCommand(.2)
        );
    }

    public Command liftToPos(LiftPresets liftPreset){return new LiftPresetCommand(lift, liftPreset);}

    public Command shootCoral(){return new CoralDispCommand(coralSubsystem, MotorDirection.FORWARD, 1);}

    public Command getCoral(){return new AutoCoralIntake(coralSubsystem);}

    public Command coralToPos(CoralPresets preset){return new CoralHozPresetCommand(coralSubsystem, preset);}

    public Command goTo(String startLoc, String endLoc){
        return new ParallelCommandGroup(
                new ToggleStop(driveSubsystem,false),
                new InstantCommand(() -> SmartDashboard.putString("TargetPath", startLoc + endLoc)),
                followPath(startLoc + endLoc),
                new ToggleStop(driveSubsystem,true));

    }

    public Command setRobotAt(String startLoc, String endLoc, LiftPresets liftLoc, CoralPresets coralLoc){
        return new ParallelCommandGroup(
                liftToPos(liftLoc),
                coralToPos(coralLoc),
                goTo(startLoc, endLoc)
        );
    }

    public Command goToAndPlace(String startLoc, String endLoc, LiftPresets liftLoc, CoralPresets coralLoc){
        return new SequentialCommandGroup(
                setRobotAt(startLoc, endLoc, liftLoc, coralLoc),
                alignToApril(limelightForwardSubsystem),
                choreo.resetOdometry(endLoc + "SourceN"), //TODO DO BETTER
                shootCoral());
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new ToggleStop(driveSubsystem, false),
//                choreo.resetOdometry("FieldRun"),
//                choreo.selectTrajectory("FieldRun"));
                choreo.resetOdometry("BargeNReefNE"),
                goToAndPlace("BargeN","ReefNE",LiftPresets.BOTTOM_REEF,CoralPresets.CENTER_POS),
                setRobotAt("ReefNE","SourceN", LiftPresets.BOTTOM, CoralPresets.CENTER_POS));

    }
}
