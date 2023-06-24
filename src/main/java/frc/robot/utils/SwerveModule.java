package frc.robot.utils;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule<T extends MotorController> {
    T driveMotor;
    T steeringMotor;
    PIDController controller;
    CANCoder encoder;
    private double lastTargetAngle;
    private double angleOffset;

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
    }

    public void drive(double speed, double targetAngle) {
        double currentAngle = encoder.getAbsolutePosition(); //TODO: update this to process the encoder value and read the actual angle of the wheel
        double desirableSetpoint = lastTargetAngle;
        if (lastTargetAngle != targetAngle) { //If the target angle has been changed since the last loop, recalculate the setpoint.
            targetAngle = targetAngle % (2 * Math.PI);
            double alternateTargetAngle = targetAngle < 0 ? targetAngle + 2 * Math.PI : targetAngle - 2 * Math.PI;
            double setpoint = targetAngle - currentAngle;
            double alternateSetpoint = alternateTargetAngle - currentAngle;
            desirableSetpoint = Math.abs(setpoint) < Math.abs(alternateSetpoint) ? setpoint : alternateSetpoint; //Finding which one would result in shorter distance travel (we don't want the wheel to go from -30 degrees to 270 degrees when it could go from -30 degrees to -90 degrees)
            angleOffset = currentAngle;
        }
        driveMotor.set(speed);
        steeringMotor.set(MathUtil.clamp(controller.calculate(currentAngle - angleOffset, desirableSetpoint), -1.0, 1.0));
        lastTargetAngle = targetAngle;
    }
}
