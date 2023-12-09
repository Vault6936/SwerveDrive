package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;

public class SwerveModule<T extends MotorController> {
    private final T driveMotor;
    private final T steeringMotor;
    private final PIDController controller;
    public final CANCoder encoder;

    public final Vector2d position;
    private boolean isCalibrating;

    private MotorDirection driveDirection = MotorDirection.FORWARD;
    private MotorDirection turnDirection = MotorDirection.FORWARD;

    public enum MotorDirection {
        FORWARD(1),
        REVERSE(-1);
        private final int direction;

        MotorDirection(int direction) {
            this.direction = direction;
        }
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains, Vector2d position, double encoderOffsetAngle) {
        this.driveMotor = driveMotor;
        if (driveMotor instanceof CANSparkMax) {
            ((CANSparkMax) driveMotor).setSmartCurrentLimit(80, 40);
            ((CANSparkMax) driveMotor).setOpenLoopRampRate(Constants.Swerve.driveRampRate);
            ((CANSparkMax) steeringMotor).setOpenLoopRampRate(Constants.Swerve.rotRampRate);
        }
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        encoder.configMagnetOffset(encoderOffsetAngle);
        this.position = position;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
        boot();
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains) {
        this(driveMotor, steeringMotor, encoder, pidGains, new Vector2d(1, 1), 0);
    }

    public void boot() {
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //Must only be set to this value!
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public void setDriveMotorDirection(MotorDirection direction) {
        driveDirection = direction;
    }

    public void setSteeringMotorDirection(MotorDirection direction) {
        turnDirection = direction;
    }

    public void calibrateWheels() {
        isCalibrating = Math.abs(drive(0, 0)) < 0.0087; //Half of a degree
    }


    public boolean doneCalibrating() {
        return isCalibrating;
    }

    public double getAngleRadians() {
        return encoder.getAbsolutePosition() / 180 * Math.PI;
    }

    private static double unsigned_0_to_2PI(double angle) {
        angle = angle % (2 * Math.PI); // The absolute value of the angle should never exceed 360 degrees
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    private static double minimumMagnitude(double... values) {
        double min = Double.POSITIVE_INFINITY;
        for (double value : values) {
            if (Math.abs(value) < Math.abs(min)) min = value;
        }
        return min;
    }

    // By always finding the minimum error, this function will avoid the type of problems that can occur when the target angle is something like 30 degrees and the wheel angle is 300.  This code would interpret the target angle as 390 degrees and calculate the error out to 90 degrees instead of 270.  Much better :)
    private static double getError(double targetAngle, double currentAngle) {
        return minimumMagnitude(targetAngle - currentAngle, targetAngle + 2 * Math.PI - currentAngle, targetAngle - 2 * Math.PI - currentAngle);
    }

    /***
     *
     * @param speed The speed to set the drive motor to.  It should be between -1.0 and 1.0.
     * @param targetAngle The desired angle, in radians, of the module.
     */
    public double drive(double speed, double targetAngle) {
        targetAngle = unsigned_0_to_2PI(targetAngle);
        double currentAngle = getAngleRadians();

        // err is how many radians the robot is off from its target angle
        double err = getError(targetAngle, currentAngle);
        double polarity = 1;
        if (Math.abs(err) > Math.PI / 2) { // Most of the time, the module will drive forward.  However, if the module is more than 90 degrees away from its target angle, it is more efficient for it to drive in reverse towards a target angle offset by 180 degrees from the original.
            err = getError((targetAngle + Math.PI) % (2 * Math.PI), currentAngle);
            polarity = -1;
        }

        driveMotor.set(MathUtil.clamp(speed * polarity * driveDirection.direction, -1.0, 1.0));
        steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0) * turnDirection.direction);

        return currentAngle;
    }

    public void rotateAndDrive(Vector2d driveVector, double rotSpeed) {
        double theta = position.angle - driveVector.angle;
        Vector2d velocityVector = new Vector2d(driveVector.magnitude - position.magnitude * rotSpeed * Math.sin(theta), rotSpeed * position.magnitude * Math.cos(theta));
        drive(velocityVector.magnitude, velocityVector.angle + driveVector.angle - Math.PI / 2);
    }
}
