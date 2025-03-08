package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    public double tx;
    public double ty;
    public double id;
    public double ry;
    public String limelightName;

    public LimelightSubsystem(String limelightName){
        this.limelightName = limelightName;
    }

    @Override
    public void periodic()
    {
        // Basic targeting data
        double tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset: crosshair to target in degrees
        double ty = LimelightHelpers.getTY(limelightName);  // Vertical offset: crosshair to target in degrees
        double id = LimelightHelpers.getFiducialID(limelightName);
        double ry = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getRotation().getY();

        this.tx = tx;
        this.ty = ty;
        this.id = id;
        this.ry = ry;

        //double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        SmartDashboard.putNumber(limelightName + " Horizontal: ", tx);
        SmartDashboard.putNumber(limelightName + " Vertical:   ", ty);
        SmartDashboard.putNumber(limelightName + " ID:         ", id);
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
