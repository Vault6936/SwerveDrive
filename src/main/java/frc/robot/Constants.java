// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.stream.FactoryConfigurationError;

public final class Constants {

    public static double deadZoneDefault;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int PAYLOAD_CONTROLLER_PORT = 1;
    }

    public static final class SwerveModuleTest {
        public static final int testModuleIndex = 1;
    }

    public static class CANIds {
        public static final double offset = 0.250;
        public static final SwerveCANId leftFront /* LEFT FRONT */ = new SwerveCANId(
                2, 3, 21,-0.227294921875 + offset); // lf module 2
        public static final SwerveCANId leftBack /* LEFT BACK */ = new SwerveCANId(
                6, 7, 23,0.145751953125+ offset);// lb module 1
        public static final SwerveCANId rightBack /* RIGHT BACK */ = new SwerveCANId(
                8, 9, 24,0.152099609375 + offset); // rb module 3
        public static final SwerveCANId rightFront /* RIGHT FRONT */  = new SwerveCANId(
                4, 5, 22,0.622802734375 + offset); // rf module 4
        public static int lift = 11;
        public static int algaeAngle = 12;
        public static int algaePusher = 13;
        public static int coralHoz = 14;
        public static int CoraldispenserSingleWheel = 15;
        public static int CoraldispenserTwoWheel = 16;
    }

    public static class SwerveCANId {
        public final int driveMotor;
        public final int steeringMotor;
        public final int encoder;
        public final double encoderOffset;

        public SwerveCANId(int driveMotor, int steeringMotor, int encoder, double encoderOffset) {
            this.driveMotor = driveMotor;
            this.steeringMotor = steeringMotor;
            this.encoder = encoder;
            this.encoderOffset = encoderOffset;
        }
    }

    public static class AnalogueSensorsIds{
        public final static int algaeAnglePot = 0;
    }

    public static class Swerve {
        public static final double driveMultiplier = 0.5;
        public static final double driveRampRate = 100.0;
        public static final double rotMultiplier = 0.5;
        public static final double rotRampRate = 75.0;
        public static final double driveMotorTicksPerRev = 1;
        public static final double GEAR_RATIO = 6.75;
        public static final double WHEEL_DIAMETER_INCHES = 4.0;

    }

    public static class SpeedConstants {
        public static final double CORAL_HOZ_SPEED = 1;
        public static final double CORAL_HOZ_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double CORAL_DISPENSER_SPEED = 1;

        public static final double LIFT_SPEED = 1;
        public static final double LIFT_SPEED_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double ALGAE_ANGLE_SPEED = 1;
        public static final double ALGAE_ANGLE_SPEED_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double ALGAE_MOVE_SPEED = .65;

        public static final double DRIVE_BASE_MAX_SPEED = .15;
    }

    public static class DebugInfo{
        public static final boolean debugDrivebase = true;
        public static final boolean debugChoreo = true;
        public static final boolean debugAlgae = false;
        public static final boolean debugCoral = false;
        public static final boolean debugCamera = true;
        public static final boolean debugAlign = false;
        public static final boolean debugLift = false;
    }

    public static class ThresholdConstants {
        public static final double ALGAE_PRESET_THRESHOLD = 10;
        public static final double CORAL_PRESET_THRESHOLD = 10;

    }

    public static class REMOVE_THIS_CLASS_PLEASE{ //TODO DELETE THIS CLASS PLEASE
        public static final boolean SLOW_MODE = true;
        public static final double slowDriveMultiplier = .3;

    }
}
