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
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //Must only be set to this value!
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }

    public void calibrateWheels() {
        isCalibrating = Math.abs(drive(0, 0)) < 0.0087; //Half of a degree
    }

    public boolean doneCalibrating() {
        return isCalibrating;
    }

    private double minimumMagnitude(double... values) {
        double min = Double.POSITIVE_INFINITY;
        for (double value : values) {
            if (Math.abs(value) < Math.abs(min)) min = value;
        }
        return min;
    }

    /***
     *
     * @param speed The speed to set the drive motor to.
     * @param targetAngle The desired angle, in radians, of the module.
     * @return The angle of the wheel in radians.
     */
    public double drive(double speed, double targetAngle) {
        targetAngle = targetAngle % (2 * Math.PI);
        if (targetAngle < 0) targetAngle += 2 * Math.PI;
        double currentAngle = encoder.getAbsolutePosition() * Math.PI / 180 - encoderOffsetAngle;
        double err = minimumMagnitude(targetAngle - currentAngle, targetAngle + 2 * Math.PI - currentAngle, targetAngle - 2 * Math.PI - currentAngle); //By always finding the minimum error, this line of code will avoid the type of problems that can occur when the target angle is something like 30 degrees and the wheel angle is 300.  This code would interpret the target angle as 390 degrees and calculate the error out to 90 degrees instead of 270.  Much better :)
        driveMotor.set(speed);
        steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0));
        return currentAngle;
    }
}
