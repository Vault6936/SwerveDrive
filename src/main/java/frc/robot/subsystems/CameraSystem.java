package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers;

public class CameraSystem  extends SubsystemBase {
    public static double tx;
    public static double ty;
    public static double id;
    public static double ry;

    @Override
    public void periodic()
    {
        // Basic targeting data
        double tx = LimelightHelpers.getTX("");  // Horizontal offset: crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset: crosshair to target in degrees
        double id = LimelightHelpers.getFiducialID("");
        double ry = LimelightHelpers.getTY("");

        CameraSystem.tx = tx;
        CameraSystem.ty = ty;
        CameraSystem.id = id;
        CameraSystem.ry = ry;

        //double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        SmartDashboard.putNumber("Horizontal: ", tx);
        SmartDashboard.putNumber("Vertical:   ", ty);
        SmartDashboard.putNumber("ID:         ", id);
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
