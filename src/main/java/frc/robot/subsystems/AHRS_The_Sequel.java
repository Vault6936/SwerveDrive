package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.math.geometry.Rotation2d;

public class AHRS_The_Sequel extends AHRS {
    private double angleAdjust;

    @Override
    public void setAngleAdjustment(double adjustment) {
        angleAdjust += (adjustment);
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getAngle() - angleAdjust);
    }
}
