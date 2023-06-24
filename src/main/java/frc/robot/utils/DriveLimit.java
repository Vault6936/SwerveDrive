package frc.robot.utils;

public interface DriveLimit {
    double getLimitedDriveValue(double currentValue, double... inputs);

    double getLimitedAccelerationValue(double lastValue, double currentValue);
}
