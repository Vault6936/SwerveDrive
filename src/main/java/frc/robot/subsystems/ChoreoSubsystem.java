package frc.robot.subsystems;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.commands.autonomousCommands.ToggleStop;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    private AutoChooser autoChooser;

    public ChoreoSubsystem(DriveSubsystem driveSubsystem)
    {
        autoFactory = new AutoFactory(() -> driveSubsystem.currentPose, driveSubsystem::poseReset, this::FollowTrajectory,
                true, driveSubsystem);
        this.autoChooser = new AutoChooser();

        this.autoChooser.addCmd("Complex auto", this::getBasicAuto);
        this.autoChooser.addCmd("Fast complex auto", this::getBasicAutoFast);
        this.autoChooser.addCmd("Forward", this::getHalfMeterForward);
        this.autoChooser.addCmd("Left", this::getHalfMeterLeft);
        this.autoChooser.addCmd("Right", this::getHalfMeterRight);

        SmartDashboard.putData(autoChooser);
        this.driveSubsystem = driveSubsystem;
        xController.setIntegratorRange(-1,1);
        yController.setIntegratorRange(-1,1);
        headingController.setIntegratorRange(-1,1);
        headingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    private void FollowTrajectory(SwerveSample sample)
    {
        Pose2d pose = driveSubsystem.currentPose;
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vy /*+ xController.calculate(pose.getX(), sample.x)*/,
                sample.vx /*+ yController.calculate(pose.getY(), sample.y)*/,
                0 /*sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading*/
        );
        driveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    private SequentialCommandGroup getBasicAuto(){
        return new SequentialCommandGroup(
                SelectTrajectory("normalAuto"),
                new ToggleStop(driveSubsystem),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem));
    }

    private SequentialCommandGroup getBasicAutoFast(){
        return new SequentialCommandGroup(
                SelectTrajectory("normalAutoFast"),
                new ToggleStop(driveSubsystem),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem));
    }

    private SequentialCommandGroup getHalfMeterForward(){
        return new SequentialCommandGroup(
                SelectTrajectory("halfMeterForward"),
                new ToggleStop(driveSubsystem),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem));
    }

    private SequentialCommandGroup getHalfMeterLeft(){
        return new SequentialCommandGroup(
                SelectTrajectory("halfMeterLeft"),
                new ToggleStop(driveSubsystem),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem));
    }
    private SequentialCommandGroup getHalfMeterRight() {
        return new SequentialCommandGroup(
                SelectTrajectory("halfMeterRight"),
                new ToggleStop(driveSubsystem),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem));
    }
    public Command SelectTrajectory(String pathName)
    {
        return autoFactory.trajectoryCmd(pathName);
    }
    public void scheduleAutoChooser(){
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    @Override
    public void periodic(){
    }
}
