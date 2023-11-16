package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import static frc.robot.Constants.Swerve;
import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;

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
    }

    public void setDriveLimit(DriveLimit driveLimit) {
        this.driveLimit = driveLimit;
    }

    public void setRotationLimit(DriveLimit rotationLimit) {
        this.rotationLimit = rotationLimit;
    }

    public void drive(double x, double y, double rot, boolean squareInputs) {
        if (squareInputs) {
            x = Math.pow(x, 2);
            y = Math.pow(y, 2);
            rot = Math.pow(rot, 2);
        }
        x *= Swerve.driveMultiplier; //Mathematically, the sqrt((ax)^2 + (ay)^2) = a * sqrt(x^2 + y^2), so the variables can be multiplied individually here
        y *= Swerve.driveMultiplier;
        rot *= Swerve.rotMultiplier;

        Vector2d inputVector = new Vector2d(new Vector2d.CartesianPoint(x, y));

        double limitedDrive = driveLimit.getLimitedDriveValue(inputVector.magnitude); //this code limits the magnitude of the drive vector
        limitedDrive = driveLimit.getLimitedAccelerationValue(limitedDrive, lastInput.vector.magnitude);
        double limitedRot = rotationLimit.getLimitedDriveValue(rot); //this limits the rotation
        limitedRot = rotationLimit.getLimitedAccelerationValue(limitedRot, lastInput.rot);

        Vector2d limitedVector = new Vector2d(new Vector2d.PolarPoint(limitedDrive, inputVector.angle)); //making another vector with the limited magnitude and the same angle

        double rotationOffset = getRotationOffset(limitedRot, inputVector);
        limitedVector = limitedVector.rotate(-pose.getRotation().getRadians());
        leftFront.drive(limitedVector.magnitude - rotationOffset, limitedVector.angle + rotationOffset * -Math.PI / 4);
        rightFront.drive(limitedVector.magnitude + rotationOffset, limitedVector.angle + rotationOffset * Math.PI / 4);
        leftBack.drive(limitedVector.magnitude - rotationOffset, limitedVector.angle + rotationOffset * Math.PI / 4);
        rightBack.drive(limitedVector.magnitude + rotationOffset, limitedVector.angle + rotationOffset * -Math.PI / 4);
        lastInput = new DriveInput(new Vector2d(new Vector2d.CartesianPoint(x, y)), limitedRot);
    }

    private static double getRotationOffset(double limitedRot, Vector2d inputVector) {
        double rotationOffset = limitedRot / (inputVector.magnitude + Math.abs(limitedRot)); //Provides a value for how much each wheel's angle should be offset from the target angle.  The offset range is -1 to 1.  With no drive input and any rotation input, the value will be 1, and with no rotation input and any drive input the value will be 0.
        if (rotationOffset == Double.POSITIVE_INFINITY)
            rotationOffset = 0; //so Java doesn't throw up at me eventually after NOT THROWING AN ARITHMETIC EXCEPTION
        return rotationOffset;
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
