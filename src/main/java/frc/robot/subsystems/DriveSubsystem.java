package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.PIDGains;
import frc.robot.swerve.SwerveChassis;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.Vector2d;
import frc.robot.webdashboard.DashboardLayout;
import frc.robot.webdashboard.WebdashboardServer;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.SwerveModuleTest.testMode;
import static frc.robot.Constants.SwerveModuleTest.testModuleIndex;

public class DriveSubsystem extends SubsystemBase {
    SwerveModule<CANSparkMax> leftFront;
    SwerveModule<CANSparkMax> rightFront;
    SwerveModule<CANSparkMax> leftBack;
    SwerveModule<CANSparkMax> rightBack;
    SwerveChassis<CANSparkMax> chassis;
    PIDGains swervePIDGains;

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        swervePIDGains = new PIDGains(0.1, 0, 0);
        leftFront = new SwerveModule<>(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftFront.encoder), swervePIDGains);
        rightFront = new SwerveModule<>(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightFront.encoder), swervePIDGains);
        leftBack = new SwerveModule<>(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.leftBack.encoder), swervePIDGains);
        rightBack = new SwerveModule<>(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANCoder(CANIds.rightBack.encoder), swervePIDGains);
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
    }

    public SwerveModule[] getModules() {
        return chassis.modules;
    }

    public void drive(double x, double y, double rot) {
        if (testMode) {
            try {
                double angle = Double.parseDouble(WebdashboardServer.getInstance(5800).getFirstConnectedLayout().getInputValue("angle")) / 180 * Math.PI;
                chassis.modules[testModuleIndex].drive((new Vector2d(x, y)).magnitude, angle);
            } catch (NumberFormatException e) {
                e.printStackTrace();
            }
        } else {
            chassis.drive(x, y, rot);
        }
    }

    @Override
    public void periodic() {
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
