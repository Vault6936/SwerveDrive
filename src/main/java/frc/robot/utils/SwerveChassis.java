package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import static frc.robot.Constants.Swerve;
import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    SwerveModule<T> leftFront;
    SwerveModule<T> rightFront;
    SwerveModule<T> leftBack;
    SwerveModule<T> rightBack;
    public SwerveChassis(SwerveModule<T> leftFront, SwerveModule<T> rightFront, SwerveModule<T> leftBack, SwerveModule<T> rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }
    public void drive(double x, double y, double rot, boolean squareInputs) {
        if (squareInputs) {
            x *= x;
            y *= y;
            rot *= rot;
        }
        x *= Swerve.driveMultiplier; //Mathematically, the sqrt((ax)^2 + (ay)^2) = a * sqrt(x^2 + y^2), so the variables can be multiplied individually here
        y *= Swerve.driveMultiplier;
        rot *= Swerve.rotMultiplier;
        double rotationOffset = rot / ((new Vector2d(new Vector2d.CartesianPoint(x, y))).magnitude + Math.abs(rot)); //Provides a value for how much each wheel's angle should be offset from the target angle.  The offset range is -1 to 1.  With no drive input and any rotation input, the value will be 1, and with no rotation input and any drive input the value will be 0.
        if (rotationOffset == Double.POSITIVE_INFINITY) rotationOffset = 0; //so Java doesn't throw up at me eventually after NOT THROWING AN ARITHMETIC EXCEPTION
        Vector2d inputVector = new Vector2d(new Vector2d.CartesianPoint(x, y));
        Vector2d outputVector = inputVector.rotate(-pose.getRotation().getRadians());
        leftFront.drive(outputVector.magnitude - rotationOffset, outputVector.angle + rotationOffset * -Math.PI / 4);
        rightFront.drive(outputVector.magnitude + rotationOffset, outputVector.angle + rotationOffset * Math.PI / 4);
        leftBack.drive(outputVector.magnitude - rotationOffset, outputVector.angle + rotationOffset * Math.PI / 4);
        rightBack.drive(outputVector.magnitude + rotationOffset, outputVector.angle + rotationOffset * -Math.PI / 4);
    }
    public void drive(double x, double y, double rot) {
        drive(x, y, rot, true);
    }
}
