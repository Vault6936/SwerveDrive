package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.PIDGains;
import frc.robot.swerve.SwerveChassis;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.Vector2d;
import frc.robot.webdashboard.DashboardLayout;

import java.util.ArrayList;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.SwerveModuleTest.testMode;
import static frc.robot.Constants.SwerveModuleTest.testModuleIndex;
import static frc.robot.GlobalVariables.pose;

public class DriveSubsystem extends SubsystemBase {
    AHRS gyro;
    final SwerveModule<SparkMax> rightBack;
    final SwerveModule<SparkMax> rightFront;
    final SwerveModule<SparkMax> leftFront;
    final SwerveModule<SparkMax> leftBack;
    public SwerveChassis<SparkMax> chassis;
    PIDGains swervePIDGains;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        swervePIDGains = new PIDGains(0.3, 0.01, 0.0005);
        rightBack = new SwerveModule<>(new SparkMax(CANIds.rightBack.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.rightBack.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightBack.encoder),
                swervePIDGains, new Vector2d(1, -1), -0.374267578125);
        rightBack.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        rightBack.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightFront = new SwerveModule<>(new SparkMax(CANIds.rightFront.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.rightFront.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightFront.encoder),
                swervePIDGains, new Vector2d(1, 1), -0.531005859375);//-0.193115234375);
        rightFront.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        rightFront.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        leftFront = new SwerveModule<>(new SparkMax(CANIds.leftFront.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.leftFront.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftFront.encoder),
                swervePIDGains, new Vector2d(-1, 1), -0.902099609375);
        leftFront.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        leftFront.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        leftBack = new SwerveModule<>(new SparkMax(CANIds.leftBack.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.leftBack.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftBack.encoder),
                swervePIDGains, new Vector2d(-1, -1),-0.900390625);
        leftBack.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        leftBack.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightBack.name = "rb";
        rightFront.name = "rf";
        leftFront.name = "lf";
        leftBack.name = "lb";
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
        gyro = new AHRS();

        xController.setIntegratorRange(-1,1);
        yController.setIntegratorRange(-1,1);
        headingController.setIntegratorRange(-1,1);

        headingController.enableContinuousInput(-Math.PI,Math.PI);

    }

    public ArrayList<SwerveModule<SparkMax>> getModules() {
        return chassis.modules;
    }



    public void drive(double x, double y, double rot) {

        if (testMode) {
            Vector2d vector = new Vector2d(x, y);
            chassis.modules.get(testModuleIndex).drive(vector.magnitude, vector.angle);
        } else {
            //DashboardLayout.setNodeValue("joystick", "x: " + x + "\ry: " + y);

            chassis.drive(-x, y, rot);


        }
    }

    public void zeroNavX() {
        gyro.zeroYaw();
    }

    public void calibrateGyro() {
        gyro.resetDisplacement();
        gyro.zeroYaw();
    }

    @Override
    public void periodic() {
        pose = new Pose2d(0, 0, gyro.getRotation2d());
        DashboardLayout.setNodeValue("encoder1", rightBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder2", rightFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder3", leftFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder4", leftBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("test mode", testMode);
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }

    public void FollowTrajectory(SwerveSample sample)
    {
        //Pose2d pose = getPos();
//        ChassisSpeeds speeds = new ChassisSpeeds(
//                sample.vx + xController.calculate(pose.getX(), sample.x),
//                sample.vy + yController.calculate(pose.getY(), sample.y),
//                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
//        );
        //driveFieldRelative(speeds);

    }

}
