package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.algaeCommands.AlgaeAnglePidControl;
import frc.robot.commands.algaeCommands.AlgaePushCommand;
import frc.robot.commands.coralCommands.CoralDispenserCommand;
import frc.robot.commands.coralCommands.CoralHozPidControl;
import frc.robot.commands.liftCommands.LiftCommand;
import frc.robot.commands.liftCommands.LiftPidControl;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);

    public final DriveSubsystem driveSubsystem;
    public final CameraSystem cameraSystem = new CameraSystem();
    //public final AlgaeEaterSystem algae = new AlgaeEaterSystem();
    //public final CoralPlacerSystem coral = new CoralPlacerSystem();
    public final LiftSubsystem lift;
    private final DriveDefaultCommand driveDefaultCommand;
    double speedMultiplier = 1;
    CoralSubsystem coralSubsystem;
    AlgaeSubsystem algaeSubsystem;
    //private final AutoFactory autoFactory; TODO add methods getPose and resetOdometry to the DriveSubsystem

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> -baseController.getLeftY(), () -> -baseController.getRightX());
        driveSubsystem.setDefaultCommand(driveDefaultCommand);

        lift = new LiftSubsystem(driveSubsystem.chassis::SetAccelerationLimit);
        lift.setDefaultCommand(new LiftPidControl(lift, () -> payloadController.getLeftY()));
        configureBindings();
        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();

        //autoFactory = new AutoFactory(driveSubsystem::getPose, driveSubsystem::resetOdometry, driveSubsystem::FollowTrajectory, true, driveSubsystem);

    }

    private void configureBindings() {
        //baseController.a().whileTrue(new AprilAlign(driveSubsystem));

        baseController.plus().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.FORWARD));
        baseController.minus().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.REVERSE));

        baseController.povLeft().whileTrue(new CoralHozPidControl(coralSubsystem, () -> 2));
        baseController.povRight().whileTrue(new CoralHozPidControl(coralSubsystem, () -> -2));


        baseController.x().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.FORWARD));
        baseController.b().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.REVERSE));

        baseController.povUp().whileTrue(new LiftPidControl(lift,() -> .5));
        baseController.povDown().whileTrue(new LiftPidControl(lift,() -> -.5));

        baseController.y().whileTrue((new LiftCommand(lift,MotorDirection.FORWARD)));
        baseController.a().whileTrue((new LiftCommand(lift,MotorDirection.REVERSE)));



//        baseController.zr().whileTrue(new LiftCommand(lift, MotorDirection.FORWARD));
//        baseController.zl().whileTrue(new LiftCommand(lift, MotorDirection.REVERSE));

        //CanID test buttons
        //baseController.a().whileTrue(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(0, true))); //Left Front
        //baseController.b().whileTrue(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(1, true))); //Right Front
        //baseController.x().whileTrue(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(2, true))); //Left Back
        //baseController.y().whileTrue(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(3, true))); //Right Back

        //baseController.a().onFalse(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(0, false)));
        //baseController.b().onFalse(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(1, false)));
        //baseController.x().onFalse(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(2, false)));
        //baseController.y().onFalse(new InstantCommand(() -> driveSubsystem.chassis.DriveMotor(3, false)));

        
        //baseController.povUp().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_1));
        //baseController.povRight().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_2));
        //baseController.povDown().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_3));
        //baseController.povLeft().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_4));

    }

    public Command getAutonomousCommand() {
        return null;
    }

}
