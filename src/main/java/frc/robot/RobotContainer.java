package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.algaeCommands.AlgaePushCommand;
import frc.robot.commands.coralCommands.CoralDispenserCommand;
import frc.robot.commands.coralCommands.CoralHozPidControl;
import frc.robot.commands.liftCommands.LiftPidControl;
import frc.robot.commands.liftCommands.LiftPresetCommand;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;
import frc.robot.swerve.SwerveModule;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandSwitchController payloadController = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);

    public final DriveSubsystem driveSubsystem;
    public final CameraSystem cameraSystem = new CameraSystem();
    public final LiftSubsystem lift;
    private final DriveDefaultCommand driveDefaultCommand;
    CoralSubsystem coralSubsystem;
    AlgaeSubsystem algaeSubsystem;
    //private final AutoFactory autoFactory; TODO add methods getPose and resetOdometry to the DriveSubsystem

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> -baseController.getLeftY(), () -> -baseController.getRightX());
//        driveDefaultCommand = new DriveDefaultCommand(
//                () -> baseController.povRight().getAsBoolean() ? 0.5 :
//                        (baseController.povLeft().getAsBoolean() ? -0.5 : 0.),
//                () -> baseController.povUp().getAsBoolean() ? 0.5 :
//                        (baseController.povDown().getAsBoolean() ? -0.5 : 0.),
//                () -> 0
//        );
        driveSubsystem.setDefaultCommand(driveDefaultCommand);

        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
        lift = new LiftSubsystem(driveSubsystem.chassis::SetAccelerationLimit, coralSubsystem, algaeSubsystem);
        lift.setDefaultCommand(new LiftPidControl(lift, () -> payloadController.getLeftY(),
                payloadController.zl().and(payloadController.zr())));

        configureBindings();

        //autoFactory = new AutoFactory(driveSubsystem::getPose, driveSubsystem::resetOdometry, driveSubsystem::FollowTrajectory, true, driveSubsystem);

    }


    private void configureBindings() {
        //baseController.a().whileTrue(new AprilAlign(driveSubsystem));

        baseController.plus().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.FORWARD));
        baseController.minus().whileTrue(new CoralDispenserCommand(coralSubsystem, MotorDirection.REVERSE));
        baseController.button(13).onTrue(new InstantCommand(() -> DriveDefaultCommand.isFieldCentric = !DriveDefaultCommand.isFieldCentric));

        baseController.povLeft().whileTrue(new CoralHozPidControl(coralSubsystem, () -> 0.5));
        baseController.povRight().whileTrue(new CoralHozPidControl(coralSubsystem, () -> -0.5));

        baseController.zr().whileTrue(new AprilAlign(driveSubsystem));
        baseController.zl().whileTrue(new AprilTurnAlign(driveSubsystem));

//        baseController.povLeft().whileTrue(new InstantCommand(() -> coralSubsystem.setHozCoral(MotorDirection.FORWARD)));
//        baseController.povRight().whileTrue(new InstantCommand(() -> coralSubsystem.setHozCoral(MotorDirection.REVERSE)));
//        baseController.povLeft().onFalse(new InstantCommand(() -> coralSubsystem.setHozCoral(MotorDirection.STOP)));
//        baseController.povRight().onFalse(new InstantCommand(() -> coralSubsystem.setHozCoral(MotorDirection.STOP)));

        baseController.x().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.FORWARD));
        baseController.y().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.REVERSE));

        baseController.a().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.FORWARD)));
        baseController.b().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.REVERSE)));
        baseController.a().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));
        baseController.b().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));

        //baseController.povUp().whileTrue(new LiftPidControl(lift,() -> .5));
        //baseController.povDown().whileTrue(new LiftPidControl(lift,() -> -.5));

        payloadController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));
        payloadController.x().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_1));
        payloadController.y().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_2));
        payloadController.b().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_3));

        //baseController.y().whileTrue((new LiftCommand(lift,MotorDirection.FORWARD)));
        //baseController.a().whileTrue((new LiftCommand(lift,MotorDirection.REVERSE)));



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
