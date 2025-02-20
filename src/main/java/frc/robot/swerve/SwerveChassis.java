package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.ArrayList;

import static frc.robot.Constants.Swerve;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;

    public final ArrayList<SwerveModule<T>> modules;

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

        // Normalize the radii
        double largest = 1.0;
        for (SwerveModule<T> module : modules) {
            if (module.position.magnitude > largest) {
                largest = module.position.magnitude;
            }
        }
        for (SwerveModule<T> module : modules) {
            module.position = new Vector2d(module.position.magnitude / largest, module.position.angle, false);
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

    public void drive(double x, double y, double rot, boolean squareInputs) {
        if (squareInputs) {
            x = Math.copySign(Math.pow(x, 2), x);
            y = Math.copySign(Math.pow(y, 2), y);
            rot = Math.copySign(Math.pow(rot, 2), rot) * -1;
        }
        double circularDivisor = getCircularDivisor(x, y); // In the joystick API, x and y can be 1 simultaneously - the inputs are bounded by driveVector.magnitude square, not a circle, so the radius can be as high as 1.41.  This converts to circular bounds.
        x /= circularDivisor;
        y /= circularDivisor;
        x *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        y *= MathUtil.clamp(Swerve.driveMultiplier, -1.0, 1.0);
        rot *= MathUtil.clamp(Swerve.rotMultiplier, -1.0, 1.0);
        Vector2d inputVector = new Vector2d(x, y);

        double divisor = Math.abs(inputVector.magnitude) + Math.abs(rot);
        if (divisor > 1.0) {
            inputVector = new Vector2d(inputVector.magnitude / divisor, inputVector.angle);
            rot /= divisor;
        }

        double limitedDrive = driveLimit.getLimitedInputValue(inputVector.magnitude);
        limitedDrive = driveLimit.getLimitedAccelerationValue(lastInput.vector.magnitude, limitedDrive);
        double limitedRot = rotationLimit.getLimitedInputValue(rot);
        limitedRot = rotationLimit.getLimitedAccelerationValue(lastInput.rot, limitedRot);
        Vector2d limitedVector = new Vector2d(limitedDrive, inputVector.angle, false); // Making another vector with the limited magnitude and the same angle
        //limitedVector = limitedVector.rotate(-pose.getRotation().getRadians()); // This is necessary for field centric drive
        // TODO: Reenable pose focus.

        for (SwerveModule<T> module : modules) {
            module.rotateAndDrive(limitedVector, limitedRot);
        }

        lastInput = new DriveInput(limitedVector, limitedRot);
    }

    public void DriveMotor(int motorNumber, boolean move)
    {
       Vector2d direction = new Vector2d(0,1);
       double speed;

       if(move)
       {
         speed = 0.5;
       }
       else
       {
           speed =0.;
       }

       if(motorNumber==0) {
           leftFront.rotateAndDrive(direction, speed);
       }
       else if (motorNumber==1) {
           rightFront.rotateAndDrive(direction, speed);
       }
       else if (motorNumber==2) {
           leftBack.rotateAndDrive(direction, speed);
       }
       else if (motorNumber==3) {
           rightBack.rotateAndDrive(direction, speed);
       }
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
