package frc.robot.swerve;


import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class SwerveModule<T extends MotorController> {
    private final T driveMotor;
    private final T steeringMotor;
    private final PIDController controller;
    public final CANcoder encoder;
    private final DoubleSupplier driveEncoder;
    public String name;

    public Vector2d position;
    private boolean isCalibrating;
    private double lastEncoderPosition;

    private MotorDirection driveDirection = MotorDirection.FORWARD;
    private MotorDirection turnDirection = MotorDirection.FORWARD;
    private double encoderOffset;
    double driveSpeedMultiplier;

    public enum MotorDirection {
        FORWARD(1),
        REVERSE(-1);
        private final int direction;

        MotorDirection(int direction) {
            this.direction = direction;
        }
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANcoder encoder, PIDGains pidGains, Vector2d position, double encoderOffset) {
        this.driveMotor = driveMotor;
        if (driveMotor instanceof SparkMax) {
            ((SparkMax) driveMotor).configure(new SparkMaxConfig().smartCurrentLimit(80, 40)
                            .openLoopRampRate(Constants.Swerve.driveRampRate),
                    SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            ((SparkMax) steeringMotor).configure(new SparkMaxConfig().smartCurrentLimit(80, 40)
                            .openLoopRampRate(Constants.Swerve.rotRampRate),
                    SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            driveEncoder = ((SparkMax) driveMotor).getEncoder()::getPosition;
        } else {
            driveEncoder = () -> 0;
        }
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
        this.position = position;
        if (Constants.REMOVE_THIS_CLASS_PLEASE.SLOW_MODE){
            this.driveSpeedMultiplier = Constants.REMOVE_THIS_CLASS_PLEASE.slowDriveMultiplier;
        }
        else this.driveSpeedMultiplier = Constants.Swerve.driveMultiplier;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
        boot();
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANcoder encoder, PIDGains pidGains) {
        this(driveMotor, steeringMotor, encoder, pidGains, new Vector2d(1, 1), 0);
    }

    public void boot() {
        encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(encoderOffset).
                withAbsoluteSensorDiscontinuityPoint(1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
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
        double rawValue = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        if (rawValue < 0) {
            rawValue += (2 * Math.PI);
        }
        SmartDashboard.putNumber("ANGLE of " + name, Math.toDegrees(rawValue));
        return rawValue;
        //return encoder.getAbsolutePosition().getValueAsDouble();//((encoder.getAbsolutePosition().getValueAsDouble() + encoderOffset) * Math.PI * 2);//.getValueAsDouble() / 180. * Math.PI;
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

    // By always finding the minimum error, this function will avoid the type of problems that can occur when the target angle
    // is something like 30 degrees and the wheel angle is 300.
    // This code would interpret the target angle as 390 degrees and calculate the error out to 90 degrees instead of 270.  Much better :)
    private static double getError(double targetAngle, double currentAngle) {
        return minimumMagnitude(targetAngle - currentAngle, targetAngle + 2 * Math.PI - currentAngle, targetAngle - 2 * Math.PI - currentAngle);
    }

    public SwerveModulePosition getOdometryData() {
        double temp = lastEncoderPosition - driveEncoder.getAsDouble();
        lastEncoderPosition = driveEncoder.getAsDouble();



        temp *= 1.15 / 39.3701;
        return new SwerveModulePosition(driveDirection.direction * temp /
                Constants.Swerve.driveMotorTicksPerRev / Constants.Swerve.GEAR_RATIO *
                Constants.Swerve.WHEEL_DIAMETER_INCHES * Math.PI,
                new Rotation2d(getAngleRadians() + Math.PI / 2));  // TODO : Decide on units and get a conversion somewhere sane.
    }

    public void slowToStop() {
        ((SparkMax) driveMotor).configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void allowMove() {
        ((SparkMax) driveMotor).configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
//        if (Math.abs(err) > Math.PI / 2) { // Most of the time, the module will drive forward.  However, if the module is more than 90 degrees away from its target angle, it is more efficient for it to drive in reverse towards a target angle offset by 180 degrees from the original.
//            err = getError((targetAngle + Math.PI) % (2 * Math.PI), currentAngle);
//            polarity = -1;
//        }

//        SmartDashboard.putNumber(name + "CurrentAngle", 180 * currentAngle / Math.PI);
        SmartDashboard.putNumber(name + "TargetAngle", 180 * targetAngle / Math.PI);
        SmartDashboard.putNumber(name + "TargetSpeed", speed * polarity * driveDirection.direction);
//        SmartDashboard.putNumber(name + "ErrAngle", 180 *  err / Math.PI);
        if (Math.abs(speed) > 0.1) {
            steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -0.4, 0.4) * turnDirection.direction);
        } else {
            steeringMotor.set(0);
        }

        if (err < Math.toRadians(5)) {
            driveMotor.set(MathUtil.clamp(
                    speed * polarity * driveDirection.direction,
                    -Constants.SpeedConstants.DRIVE_BASE_MAX_SPEED, Constants.SpeedConstants.DRIVE_BASE_MAX_SPEED));
        } else {
            driveMotor.set(0);
        }
        //SmartDashboard.putNumber(name + "TargetPower", MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0) * turnDirection.direction);

        return currentAngle;
    }

    public void rotateAndDrive(Vector2d driveVector, double rotSpeed) {
        Vector2d rotationVector = position.rotate(Math.PI / 2).multiply(rotSpeed);
        Vector2d velocityVector = new Vector2d(rotationVector.x + driveVector.x,rotationVector.y + driveVector.y);
        SmartDashboard.putNumber(name + "RotationVector", Math.toDegrees(rotationVector.angle));
        SmartDashboard.putNumber(name + "VelocityVector", Math.toDegrees(velocityVector.angle));

        drive(velocityVector.magnitude, velocityVector.angle);
    }
}