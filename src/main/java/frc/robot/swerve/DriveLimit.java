package frc.robot.swerve;

public interface DriveLimit {
    double getLimitedDriveValue(double currentValue, double... inputs);

    double getLimitedAccelerationValue(double lastValue, double currentValue);
}