package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AHRS_The_Sequel extends AHRS {
    private double angleAdjust;

    @Override
    public void setAngleAdjustment(double adjustment) {
        angleAdjust = (adjustment);
    }

    @Override
    public Rotation2d getRotation2d() {
        SmartDashboard.putNumber("RawRotation", -getAngle());
        SmartDashboard.putNumber("AngleAdjust", angleAdjust);
        return Rotation2d.fromDegrees(-getAngle() - angleAdjust);
    }

    public Rotation2d getRawAngle()
    {
        return Rotation2d.fromDegrees(-getAngle());
    }
}
