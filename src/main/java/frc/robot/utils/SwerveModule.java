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

    public SwerveModule(T driveMotor, T steeringMotor, CANCoder encoder, PIDGains pidGains) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
    }

    //TODO: What happens to the PID controller when the angle crosses from -15 degrees to +27 degrees?  Find a workaround.
    public void drive(double speed, double targetAngle) { //If the allowed angle range is -360 to 360 degrees, then 1 angle could be expressed in 2 ways (e.g. -90 and 270).  This function finds the correct way to express the angle such that the wheel won't have to turn more than it has to.
        double currentAngle = encoder.getAbsolutePosition();
        targetAngle = targetAngle % (2 * Math.PI);
        double alternateTargetAngle = targetAngle < 0 ? targetAngle + 2 * Math.PI : targetAngle - 2 * Math.PI;
        double distance = Math.abs(targetAngle - currentAngle);
        double alternateDistance = Math.abs(alternateTargetAngle - currentAngle);
        targetAngle = distance < alternateDistance ? targetAngle : alternateTargetAngle;
        steeringMotor.set(MathUtil.clamp(controller.calculate(currentAngle, targetAngle), -1.0, 1.0));
        driveMotor.set(speed);
    }
}
