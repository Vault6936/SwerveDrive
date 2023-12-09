package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.ArrayList;

import static frc.robot.Constants.Swerve;
import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;

    public final ArrayList<SwerveModule<T>> modules;

    protected static double rotationConstant;

    public static final class DriveLimits {
        public static final InputLimit NONE = new InputLimit() {
            @Override
            public double getLimitedInputValue(double currentValue, double... inputs) {
                return currentValue;
            }

            @Override
            public double getLimitedAccelerationValue(double lastValue, double currentValue) {
                return currentValue;
            }
        };
    }

    private InputLimit driveLimit = DriveLimits.NONE;
    private InputLimit rotationLimit = DriveLimits.NONE;

    private DriveInput lastInput = new DriveInput(new Vector2d(), 0);

    public SwerveChassis(SwerveModule<T> leftFront, SwerveModule<T> rightFront, SwerveModule<T> leftBack, SwerveModule<T> rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        modules = new ArrayList<>();
        modules.add(leftFront);
        modules.add(rightFront);
        modules.add(leftBack);
        modules.add(rightBack);
        for (SwerveModule<T> module : modules) {
            double max = 1 + module.position.magnitude;
            if (max > rotationConstant) {
                rotationConstant = max;
            }
        }
    }

    public void setDriveLimit(InputLimit inputLimit) {
        this.driveLimit = inputLimit;
    }

    public void setRotationLimit(InputLimit rotationLimit) {
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
        double divisor = getCircularDivisor(x, y); // In the joystick API, x and y can be 1 simultaneously - the inputs are bounded by driveVector.magnitude square, not a circle, so the radius can be as high as 1.41.  This converts to circular bounds.
        x /= divisor;
        y /= divisor;
        x *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        y *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        rot *= MathUtil.clamp(Swerve.rotMultiplier, -1.0, 1.0);
        Vector2d inputVector = new Vector2d(x, y);

        double limitedDrive = driveLimit.getLimitedInputValue(inputVector.magnitude);
        limitedDrive = driveLimit.getLimitedAccelerationValue(lastInput.vector.magnitude, limitedDrive);
        double limitedRot = rotationLimit.getLimitedInputValue(rot);
        limitedRot = rotationLimit.getLimitedAccelerationValue(lastInput.rot, limitedRot);
        Vector2d limitedVector = new Vector2d(limitedDrive, inputVector.angle, false); // Making another vector with the limited magnitude and the same angle
        limitedVector = limitedVector.rotate(-pose.getRotation().getRadians()); // This is necessary for field centric drive

        double speedDivisor = 1.0;
        double moduleDriveValues[][] = new double[][]{{}, {}, {}, {}};
        for (int i = 0; i < modules.size(); i++) {
            moduleDriveValues[i] = modules.get(i).calculateMixedDrive(limitedVector, limitedRot);
            if (moduleDriveValues[i][0] > speedDivisor) {
                speedDivisor = moduleDriveValues[i][0];
            }
        }
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).drive(moduleDriveValues[i][0] / speedDivisor, moduleDriveValues[i][1]);
        }

        lastInput = new DriveInput(limitedVector, limitedRot);
    }

    public void drive(double x, double y, double rot) {
        drive(x, y, rot, true);
    }

    private static class DriveInput {
        final Vector2d vector;
        final double rot;

        DriveInput(Vector2d driveVector, double rot) {
            this.vector = driveVector;
            this.rot = rot;
        }
    }
}
