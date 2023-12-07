package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.webdashboard.DashboardLayout;

import static frc.robot.Constants.Swerve;
import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;

    public final SwerveModule[] modules;

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

    private boolean lastRotated = false;

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

    private static double getCircularDivisor(double x, double y) {
        if (x == 0.0 && y == 0.0) {
            return 1.0;
        } else if (Math.abs(y / x) >= 1.0) {
            return Math.sqrt(1 + Math.pow(x / y, 2));
        } else {
            return Math.sqrt(1 + Math.pow(y / x, 2));
        }
    }

    private static int polarity(double num) {
        return num < 0 ? -1 : 1;
    }

    public void drive(double x, double y, double rot, boolean squareInputs) {
        if (squareInputs) {
            x = Math.pow(x, 2) * polarity(x);
            y = Math.pow(y, 2) * polarity(y);
            rot = Math.pow(rot, 2) * polarity(rot) * -1;
        }
        double divisor = getCircularDivisor(x, y); // In the joystick API, x and y can be 1 simultaneously - the inputs are bounded by a square, not a circle, so the radius can be as high as 1.41.  This converts to circular bounds.
        x /= divisor;
        y /= divisor;
        x *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        y *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        rot *= MathUtil.clamp(Swerve.rotMultiplier, -1.0, 1.0);
        Vector2d inputVector = new Vector2d(x, y);

        double limitedDrive = driveLimit.getLimitedDriveValue(inputVector.magnitude);
        limitedDrive = driveLimit.getLimitedAccelerationValue(lastInput.vector.magnitude, limitedDrive);
        double limitedRot = rotationLimit.getLimitedDriveValue(rot);
        limitedRot = rotationLimit.getLimitedAccelerationValue(lastInput.rot, limitedRot);
        Vector2d limitedVector = new Vector2d(limitedDrive, inputVector.angle - Math.PI / 2, false); // Making another vector with the limited magnitude and the same angle

        double rotPercent = Math.abs(limitedRot) / (inputVector.magnitude + Math.abs(limitedRot)); // Provides a value for how much each wheel's angle should be offset from the target angle.  The offset range is -1 to 1.  With no drive input and any rotation input, the value will be 1, and with no rotation input and any drive input the value will be 0.
        if (Math.abs(rotPercent) == Double.POSITIVE_INFINITY || Double.isNaN(rotPercent))
            rotPercent = 0;

        limitedVector = limitedVector.rotate(-pose.getRotation().getRadians()); // This is necessary for field centric drive

        if (rotPercent == 0.0) {
            leftFront.drive(limitedVector.magnitude - limitedRot, limitedVector.angle);
            rightFront.drive(limitedVector.magnitude + limitedRot, limitedVector.angle);
            leftBack.drive(limitedVector.magnitude - limitedRot, limitedVector.angle);
            rightBack.drive(limitedVector.magnitude + limitedRot, limitedVector.angle);

        } else if (rotPercent <= 1.0) {
            for (SwerveModule module : modules) {
                rotateWhileDriving(module, limitedVector.rotate(Math.PI / 2), limitedRot);
            }
        } else {
            leftFront.drive(limitedVector.magnitude - limitedRot, leftFront.fullRotAngle);
            rightFront.drive(limitedVector.magnitude + limitedRot, rightFront.fullRotAngle);
            leftBack.drive(limitedVector.magnitude - limitedRot, leftBack.fullRotAngle);
            rightBack.drive(limitedVector.magnitude + limitedRot, rightBack.fullRotAngle);
            lastRotated = true;
        }
        lastInput = new DriveInput(limitedVector, limitedRot);
    }

    private static void rotateWhileDriving(SwerveModule module, Vector2d driveVector, double rotSpeed) {
        double a = driveVector.magnitude;
        double b = rotSpeed;
        double r = module.position.magnitude;
        double theta = module.position.angle - driveVector.angle;
        Vector2d velocityVector = new Vector2d(a - r * b * Math.sin(theta), b * r * Math.cos(theta));
        module.drive(velocityVector.magnitude / 1.414, velocityVector.angle + driveVector.angle - Math.PI / 2);
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
