package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class DriveSubsystem extends SubsystemBase {
    AHRS gyro;
    final SwerveModule<SparkMax> rightBack;
    final SwerveModule<SparkMax> rightFront;
    final SwerveModule<SparkMax> leftFront;
    final SwerveModule<SparkMax> leftBack;
    public SwerveChassis<SparkMax> chassis;
    PIDGains swervePIDGains;
    private Pose2d currentPose;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        gyro = new AHRS();
        currentPose = new Pose2d(0.0,0.0,gyro.getRotation2d());
        swervePIDGains = new PIDGains(0.3, 0.01, 0.0005);
        rightBack = new SwerveModule<>(new SparkMax(CANIds.rightBack.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.rightBack.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightBack.encoder),
                swervePIDGains, new Vector2d(1, -1), CANIds.rightBack.encoderOffset);//-0.374267578125);
        rightBack.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        rightBack.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightFront = new SwerveModule<>(new SparkMax(CANIds.rightFront.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.rightFront.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightFront.encoder),
                swervePIDGains, new Vector2d(1, 1), CANIds.rightFront.encoderOffset);//-0.531005859375);//-0.193115234375);
        rightFront.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        rightFront.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        leftFront = new SwerveModule<>(new SparkMax(CANIds.leftFront.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.leftFront.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftFront.encoder),
                swervePIDGains, new Vector2d(-1, 1),CANIds.leftFront.encoderOffset);
        leftFront.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        leftFront.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        leftBack = new SwerveModule<>(new SparkMax(CANIds.leftBack.driveMotor, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(CANIds.leftBack.steeringMotor, SparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftBack.encoder),
                swervePIDGains, new Vector2d(-1, -1),CANIds.leftBack.encoderOffset);//-0.900390625);
        leftBack.setSteeringMotorDirection(SwerveModule.MotorDirection.FORWARD);
        leftBack.setDriveMotorDirection(SwerveModule.MotorDirection.REVERSE);
        rightBack.name = "rb";
        rightFront.name = "rf";
        leftFront.name = "lf";
        leftBack.name = "lb";
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);

        xController.setIntegratorRange(-1,1);
        yController.setIntegratorRange(-1,1);

        headingController.setIntegratorRange(-1,1);
        headingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    public ArrayList<SwerveModule<SparkMax>> getModules() {
        return chassis.modules;
    }

    public void drive(double x, double y, double rot)
    {
        drive(x, y, rot, false);
    }

    public void drive(double x, double y, double rot, boolean fieldCentric) {

        if (testMode) {
            Vector2d vector = new Vector2d(x, y);
            chassis.modules.get(testModuleIndex).drive(vector.magnitude, vector.angle);
        } else {
            //DashboardLayout.setNodeValue("joystick", "x: " + x + "\ry: " + y);
            if(fieldCentric)
            {
                Vector2d driveDirection = new Vector2d(-x, y);
                SmartDashboard.putString("Drive Raw ", driveDirection.y + "/" + driveDirection.x);
                driveDirection = driveDirection.rotate(gyro.getRotation2d().getRadians());
                SmartDashboard.putString("Drive Corrected ", driveDirection.y + "/" + driveDirection.x);
                chassis.drive(driveDirection.x, driveDirection.y, rot);
            }
            else
            {
                chassis.drive(-x, y, rot);
            }
            SmartDashboard.putBoolean("Field Centric: ", fieldCentric);
        }
    }

    private void updatePose(SwerveModule<SparkMax> lf, SwerveModule<SparkMax> lb, SwerveModule<SparkMax> rb, SwerveModule<SparkMax> rf){
//        double magnitude = (lf.getOdometryData().distanceMeters + lb.getOdometryData().distanceMeters + rb.getOdometryData().distanceMeters + rf.getOdometryData().distanceMeters) /4;
//        double angle = (lf.getAngleRadians() + lb.getAngleRadians() + rb.getAngleRadians() +rf.getAngleRadians()) / 4;
//        double poseX = currentPose.getX() + magnitude * Math.cos(angle);
//        double poseY = currentPose.getY() + magnitude * Math.sin(angle);
        SwerveModulePosition lf_od = lf.getOdometryData();
        SwerveModulePosition lb_od = lb.getOdometryData();
        SwerveModulePosition rf_od = rf.getOdometryData();
        SwerveModulePosition rb_od = rb.getOdometryData();

        double lf_x = lf_od.distanceMeters * Math.cos(lf_od.angle.getRadians());
        double lb_x = lb_od.distanceMeters * Math.cos(lb_od.angle.getRadians());
        double rf_x = rf_od.distanceMeters * Math.cos(rf_od.angle.getRadians());
        double rb_x = rb_od.distanceMeters * Math.cos(rb_od.angle.getRadians());
        double lf_y = lf_od.distanceMeters * Math.sin(lf_od.angle.getRadians());
        double lb_y = lb_od.distanceMeters * Math.sin(lb_od.angle.getRadians());
        double rf_y = rf_od.distanceMeters * Math.sin(rf_od.angle.getRadians());
        double rb_y = rb_od.distanceMeters * Math.sin(rb_od.angle.getRadians());

        double poseX = (lf_x + lb_x + rf_x + rb_x) / 4.0 ;
        double poseY = (lf_y + lb_y + rf_y + rb_y) / 4.0 ;

        double turn_mag = (poseX / Math.abs(poseX)) * Math.sqrt(Math.pow(poseX,2) + Math.pow(poseY,2));
        double turn_angle;
        if (poseX == 0) {
            turn_angle = gyro.getRotation2d().getRadians();
        } else {
            turn_angle = gyro.getRotation2d().getRadians() + Math.atan(poseY / poseX);
        }

        double turn_poseX = currentPose.getX() + turn_mag * Math.cos(turn_angle);
        double turn_poseY = currentPose.getY() + turn_mag * Math.sin(turn_angle);
        currentPose = new Pose2d(turn_poseX, turn_poseY, gyro.getRotation2d());
        SmartDashboard.putNumber("Turn Mag: ", turn_mag);
        SmartDashboard.putNumber("Turn Angle: ", turn_angle);
        SmartDashboard.putNumber("PoseX: ", turn_poseX);
        SmartDashboard.putNumber("PoseY: ", turn_poseY);
        SmartDashboard.putString("Current Pose:", currentPose.toString());

    }

    public void poseReset(){
        currentPose = new Pose2d(0,0,gyro.getRotation2d());
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
        updatePose(leftFront,leftBack,rightBack,rightFront);
        DashboardLayout.setNodeValue("encoder1", rightBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder2", rightFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder3", leftFront.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("encoder4", leftBack.encoder.getAbsolutePosition());
        DashboardLayout.setNodeValue("test mode", testMode);
        SmartDashboard.putNumber("LeftFrontEncoderDiff", leftFront.getOdometryData().distanceMeters);
        SmartDashboard.putNumber("LeftBackEncoderDiff", leftBack.getOdometryData().distanceMeters);
        SmartDashboard.putNumber("RightFrontEncoderDiff", rightFront.getOdometryData().distanceMeters);
        SmartDashboard.putNumber("RightBackEncoderDiff", rightBack.getOdometryData().distanceMeters);
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
