package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import static frc.robot.Constants.Swerve;
import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;

    public final SwerveModule<T>[] modules;

    public static final class DriveLimits {
        public static final DriveLimit NONE = new DriveLimit() {
            @Override
            public double getLimitedDriveValue(double currentValue, double... inputs) {
                return currentValue;
            }

            @Override
            public double getLimitedAccelerationValue(double lastValue, double currentValue) {
                return currentValue;
            }
        };
    }

    private DriveLimit driveLimit = DriveLimits.NONE;
    private DriveLimit rotationLimit = DriveLimits.NONE;

    private DriveInput lastInput = new DriveInput(new Vector2d(), 0);

    public SwerveChassis(SwerveModule<T> leftFront, SwerveModule<T> rightFront, SwerveModule<T> leftBack, SwerveModule<T> rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        modules = new SwerveModule[]{leftFront, rightFront, leftBack, rightBack};
    }

    public void setDriveLimit(DriveLimit driveLimit) {
        this.driveLimit = driveLimit;
    }

    public void setRotationLimit(DriveLimit rotationLimit) {
        this.rotationLimit = rotationLimit;
    }

    private double getAverageAngle() {
        double average = 0;
        for (SwerveModule module : modules) {
            average += module.getAngleRadians();
        }
        return average / 4;
    }

    public void drive(double x, double y, double rot, boolean squareInputs) {
        if (squareInputs) {
            x = Math.pow(x, 2);
            y = Math.pow(y, 2);
            rot = Math.pow(rot, 2);
        }

        x *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0); // Mathematically, the sqrt((ax)^2 + (ay)^2) = a * sqrt(x^2 + y^2), so the variables can be multiplied individually here
        y *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        rot *= MathUtil.clamp(Swerve.rotMultiplier, -1.0, 1.0);
        Vector2d inputVector = new Vector2d(x, y);

        double limitedDrive = driveLimit.getLimitedDriveValue(inputVector.magnitude); // This code limits the magnitude of the drive vector
        limitedDrive = driveLimit.getLimitedAccelerationValue(limitedDrive, lastInput.vector.magnitude);
        double limitedRot = rotationLimit.getLimitedDriveValue(rot); //this limits the rotation
        limitedRot = rotationLimit.getLimitedAccelerationValue(limitedRot, lastInput.rot);
        Vector2d limitedVector = new Vector2d(limitedDrive, inputVector.angle, false); // Making another vector with the limited magnitude and the same angle

        double rotationOffset = limitedRot / (inputVector.magnitude + Math.abs(limitedRot)); // Provides a value for how much each wheel's angle should be offset from the target angle.  The offset range is -1 to 1.  With no drive input and any rotation input, the value will be 1, and with no rotation input and any drive input the value will be 0.
        if (rotationOffset == Double.POSITIVE_INFINITY)
            rotationOffset = 0;
        limitedVector = limitedVector.rotate(-pose.getRotation().getRadians()); // This is necessary for field centric drive
        leftFront.drive(limitedVector.magnitude - limitedRot, limitedVector.angle + rotationOffset * leftFront.fullRotAngle);
        rightFront.drive(limitedVector.magnitude + limitedRot, limitedVector.angle + rotationOffset * rightFront.fullRotAngle);
        leftBack.drive(limitedVector.magnitude - limitedRot, limitedVector.angle + rotationOffset * leftBack.fullRotAngle);
        rightBack.drive(limitedVector.magnitude + limitedRot, limitedVector.angle + rotationOffset * rightBack.fullRotAngle);
        lastInput = new DriveInput(new Vector2d(x, y), limitedRot);
    }

    public void drive(double x, double y, double rot) {
        drive(x, y, rot, true);
    }

    private static class DriveInput {
        final Vector2d vector;
        final double rot;

        DriveInput(Vector2d magnitude, double rot) {
            this.vector = magnitude;
            this.rot = rot;
        }
    }
}
