package frc.robot.swerve;

public interface InputLimit {
    double getLimitedInputValue(double currentValue, double... inputs);

    double getLimitedAccelerationValue(double lastValue, double currentValue);
}