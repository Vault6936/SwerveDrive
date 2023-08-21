package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule<T extends MotorController> {
    private final T driveMotor;
    private final T steeringMotor;
    private final PIDController controller;
    private final CANCoder encoder;
    private final double encoderOffset;
    private double lastTargetAngle;
    private double angleOffset;
    private boolean isCalibrating;

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
        this.encoderOffset = encoderOffset;
        boot();
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains) {
        this(driveMotor, steeringMotor, encoder, pidGains, 0);
    }

    public void boot() {
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }

    public void calibrateWheels() {
        isCalibrating = Math.abs(drive(0, 0)) < 0.0087;
    }

    public boolean doneCalibrating() {
        return isCalibrating;
    }

    public double drive(double speed, double targetAngle) {
        double currentAngle = encoder.getAbsolutePosition() * Math.PI / 180 - encoderOffset;
        double desirableSetpoint = lastTargetAngle;
        if (lastTargetAngle != targetAngle) { //If the target angle has been changed since the last loop, recalculate the setpoint.
            targetAngle = targetAngle % (2 * Math.PI);
            double alternateTargetAngle = targetAngle < 0 ? targetAngle + 2 * Math.PI : targetAngle - 2 * Math.PI;
            double setpoint = targetAngle - currentAngle;
            double alternateSetpoint = alternateTargetAngle - currentAngle;
            desirableSetpoint = Math.abs(setpoint) < Math.abs(alternateSetpoint) ? setpoint : alternateSetpoint; //Finding which one would result in shorter distance travel (we don't want the wheel to go from -30 degrees to 270 degrees when it could go from -30 degrees to -90 degrees)
            angleOffset = currentAngle; //This is done to avoid problems when the angle crosses from, say, -179 degrees to 181 degrees or vice versa.  I suppose if I set the absolute encoder range from 0 to 360 then this would be unnecessary, but it's here anyway.
        }
        driveMotor.set(speed);
        steeringMotor.set(MathUtil.clamp(controller.calculate(currentAngle - angleOffset, desirableSetpoint), -1.0, 1.0));
        lastTargetAngle = targetAngle;
        return currentAngle;
    }
}
