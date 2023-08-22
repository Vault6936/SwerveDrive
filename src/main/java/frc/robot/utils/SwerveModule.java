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
    private final double encoderOffsetAngle;
    private boolean isCalibrating;

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains, double encoderOffsetAngle) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
        this.encoderOffsetAngle = encoderOffsetAngle;
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
        double currentAngle = encoder.getAbsolutePosition() * Math.PI / 180 - encoderOffsetAngle;
        double err = Math.min(Math.min(targetAngle - currentAngle, targetAngle + 2 * Math.PI - currentAngle), targetAngle - 2 * Math.PI - currentAngle);
        driveMotor.set(speed);
        steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0));
        return currentAngle;
    }
}
