package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
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
    ChoreoSubsystem choreo;

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> -baseController.getLeftY(), () -> -(-baseController.getRightX()));
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        choreo = new ChoreoSubsystem(driveSubsystem);
        coralSubsystem = new CoralSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
        lift = new LiftSubsystem(driveSubsystem.chassis::SetAccelerationLimit, coralSubsystem, algaeSubsystem);
        lift.setDefaultCommand(new LiftPidControl(lift, () -> payloadController.getLeftY(),
                payloadController.zl().and(payloadController.zr())));

        configureBindings();
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

        baseController.x().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.FORWARD));
        baseController.y().whileTrue(new AlgaePushCommand(algaeSubsystem,MotorDirection.REVERSE));

        baseController.a().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.FORWARD)));
        baseController.b().whileTrue(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.REVERSE)));
        baseController.a().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));
        baseController.b().onFalse(new InstantCommand(()->algaeSubsystem.setAngleAlgae(MotorDirection.STOP)));

        payloadController.a().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_0));
        payloadController.x().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_1));
        payloadController.y().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_2));
        payloadController.b().whileTrue(new LiftPresetCommand(lift, LiftPresets.POSITION_3));

        //baseController.y().whileTrue((new LiftCommand(lift,MotorDirection.FORWARD)));
        //baseController.a().whileTrue((new LiftCommand(lift,MotorDirection.REVERSE)));



//        baseController.zr().whileTrue(new LiftCommand(lift, MotorDirection.FORWARD));
//        baseController.zl().whileTrue(new LiftCommand(lift, MotorDirection.REVERSE));

    }

    public Command getAutonomousCommand() {
        return choreo.SelectTrajectory("NewPath");
    }
}
