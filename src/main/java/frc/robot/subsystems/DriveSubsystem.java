package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.*;
import frc.robot.webdashboard.DashboardLayout;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.SwerveModuleTest.testMode;
import static frc.robot.Constants.SwerveModuleTest.testModuleIndex;
import static frc.robot.GlobalVariables.pose;

public class DriveSubsystem extends SubsystemBase {
    AHRS gyro;
    final SwerveModule<CANSparkMax> leftFront;
    final SwerveModule<CANSparkMax> rightFront;
    final SwerveModule<CANSparkMax> leftBack;
    final SwerveModule<CANSparkMax> rightBack;
    SwerveChassis<CANSparkMax> chassis;
    PIDGains swervePIDGains;

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        swervePIDGains = new PIDGains(0.3, 0.01, 0.0005);
        leftFront = new SwerveModule<>(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftFront.encoder), swervePIDGains, new Vector2d(-1, 1), 45 - 3.07);
        leftFront.setSteeringMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightFront = new SwerveModule<>(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightFront.encoder), swervePIDGains, new Vector2d(1, 1), 110 + 0.615);
        rightFront.setSteeringMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightFront.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        leftBack = new SwerveModule<>(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftBack.encoder), swervePIDGains, new Vector2d(-1, -1), 135 + 7.207);
        leftBack.setSteeringMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightBack = new SwerveModule<>(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightBack.encoder), swervePIDGains, new Vector2d(1, -1), 145 - 1.582);
        rightBack.setSteeringMotorDirection(SwerveModule.MotorDirection.REVERSE);
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
        gyro = new AHRS();
    }

    public SwerveModule[] getModules() {
        return chassis.modules;
    }

    public void drive(double x, double y, double rot) {

        if (testMode) {
            Vector2d vector = new Vector2d(x, y);
            chassis.modules[testModuleIndex].drive(vector.magnitude, vector.angle);
        } else {
            //DashboardLayout.setNodeValue("joystick", "x: " + x + "\ry: " + y);
            chassis.drive(x, -y, rot);
        }
    }

    public void zeroNavX() {
        gyro.zeroYaw();
        gyro.calibrate();
    }

    @Override
    public void periodic() {
        pose = new Pose2d(0, 0, gyro.getRotation2d());
        DashboardLayout.setNodeValue("encoder1", leftFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder2", rightFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder3", leftBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder4", rightBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("test mode", testMode);
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }
}
