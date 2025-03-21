package frc.robot.subsystems;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.autonomousCommands.ToggleStop;

public class ChoreoSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory; //TODO add methods getPose and resetOdometry to the DriveSubsystem
    private final DriveSubsystem driveSubsystem;

    private final PIDController xController = new PIDController(1.0/2, 0.1, 0.0);
    private final PIDController yController = new PIDController(1.0/2, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1.0, 0.0, 0.0);
    private AutoChooser autoChooser;

    public ChoreoSubsystem(DriveSubsystem driveSubsystem)
    {
        autoFactory = new AutoFactory(() -> driveSubsystem.currentPose, driveSubsystem::poseReset, this::FollowTrajectory,
                false, driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        xController.setIntegratorRange(-50, 50);
        yController.setIntegratorRange(-50, 50);
        headingController.setIntegratorRange(-6, 6);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void FollowTrajectory(SwerveSample sample)
    {
        double forwardCalc = -xController.calculate(driveSubsystem.currentPose.getX(), sample.x);
        double leftCalc = -yController.calculate(driveSubsystem.currentPose.getY(), sample.y);
        double headingCalc = headingController.calculate(driveSubsystem.currentPose.getRotation().getRadians(), sample.heading);

//        ChassisSpeeds speeds = new ChassisSpeeds(
//                -sample.vy, //+ forwardCalc,
//                sample.vx, // + leftCalc,
//                0 /*sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading*/
//        );

        if(Constants.DebugInfo.debugChoreo) {
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
                    MathUtil.clamp( -sample.omega - headingCalc, -0.5, 0.5),
                    true);
    }

    private SequentialCommandGroup Example() {
        return new SequentialCommandGroup(
                selectTrajectory("halfMeterRight"),
                new ToggleStop(driveSubsystem, true),
                new WaitCommand(0.2),
                new ToggleStop(driveSubsystem, false));
    }

    public Command resetOdometry(String pathName){
        return autoFactory.resetOdometry(pathName);
    }

    public Command selectTrajectory(String pathName)
    {
        return autoFactory.trajectoryCmd(pathName);
    }
    public void scheduleAutoChooser(){
        //RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    @Override
    public void periodic(){
    }
}
