package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDGains;
import frc.robot.utils.SwerveChassis;
import frc.robot.utils.SwerveModule;

import static frc.robot.Constants.CANIds;

public class DriveSubsystem extends SubsystemBase {
    SwerveModule<CANSparkMax> leftFront;
    SwerveModule<CANSparkMax> rightFront;
    SwerveModule<CANSparkMax> leftBack;
    SwerveModule<CANSparkMax> rightBack;
    SwerveChassis<CANSparkMax> chassis;
    PIDGains swervePIDGains;

    public DriveSubsystem() {
        swervePIDGains = new PIDGains(0.1, 0, 0);
        leftFront = new SwerveModule<>(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftFront.encoder), swervePIDGains);
        rightFront = new SwerveModule<>(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightFront.encoder), swervePIDGains);
        leftBack = new SwerveModule<>(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftBack.encoder), swervePIDGains);
        rightBack = new SwerveModule<>(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightBack.encoder), swervePIDGains);
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
    }

    public void drive(double x, double y, double rot) {
        chassis.drive(x, y, rot);
    }
}
