package frc.robot.subsystems.Other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    public double tz;
    public double tx;
    public double ty;
    public double id;
    public double ry;
    public final String limelightName;
    public final double flushOffset;

    public Pose2d fieldLoc = new Pose2d();

    public LimelightSubsystem(String limelightName, double flushOffset){
        this.limelightName = "limelight-" + limelightName;
        this.flushOffset = flushOffset;
    }

    @Override
    public void periodic()
    {
        // Basic targeting data
        double tz = -LimelightHelpers.getCameraPose3d_TargetSpace(limelightName).getZ();
        double tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset: crosshair to target in degrees
        double ty = LimelightHelpers.getTY(limelightName);  // Vertical offset: crosshair to target in degrees
        double id = LimelightHelpers.getFiducialID(limelightName);
        double ry = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getY();

        this.fieldLoc = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);

        this.tz = tz;
        this.tx = tx;
        this.ty = ty;
        this.id = id;
        this.ry = ry;

        //double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        if (Constants.DebugInfo.debugCamera) {
            SmartDashboard.putNumber(limelightName + " Horizontal: ", tx);
            SmartDashboard.putNumber(limelightName + " Vertical:   ", ty);
            SmartDashboard.putNumber(limelightName + " ID:         ", id);
            SmartDashboard.putNumber(limelightName + "tz", tz);
        }
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
