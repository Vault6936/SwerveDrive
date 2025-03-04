package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public ChoreoSubsystem(DriveSubsystem driveSubsystem)
    {
        autoFactory = new AutoFactory(() -> driveSubsystem.currentPose, driveSubsystem::poseReset, this::FollowTrajectory,
                true, driveSubsystem);
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

    public Command SelectTrajectory(String pathName)
    {
        return autoFactory.trajectoryCmd(pathName);
    }
}
