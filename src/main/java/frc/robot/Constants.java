// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static double deadZoneDefault;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int PAYLOAD_CONTROLLER_PORT = 1;
    }

    public static final class SwerveModuleTest {
        public static final boolean testMode = false;

        public static final int testModuleIndex = 1;
    }

    public static class CANIds {
        public static final SwerveCANId leftFront = new SwerveCANId(4, 5, 22); // module 2
        public static final SwerveCANId rightFront = new SwerveCANId(2, 3, 21); // module 1
        public static final SwerveCANId leftBack = new SwerveCANId(6, 7, 23); // module 3
        public static final SwerveCANId rightBack = new SwerveCANId(8, 9, 24); // module 4
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

        public SwerveCANId(int driveMotor, int steeringMotor, int encoder) {
            this.driveMotor = driveMotor;
            this.steeringMotor = steeringMotor;
            this.encoder = encoder;
        }
    }

    public static class Swerve {
        public static final double driveMultiplier = 0.5;
        public static final double driveRampRate = 100.0;
        public static final double rotMultiplier = 1;
        public static final double rotRampRate = 75.0;
        public static final double SPEED_OF_APRILALIGN = .5;
        public static final double driveMotorTicksPerRev = 1;
        public static final double GEAR_RATIO = 6.75;
        public static final double WHEEL_DIAMETER_INCHES = 4.0;

    }

    public static class SpeedConstants { //TODO REMOVE. I DID NOT KNOW WHERE TO PUT ALL THESE SPEED CONSTANTS
        public static final double CORAL_HOZ_SPEED = 1;
        public static final double CORAL_HOZ_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double CORAL_DISPENSER_SPEED = 1;

        public static final double LIFT_SPEED = 1;
        public static final double LIFT_SPEED_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double ALGAE_ANGLE_SPEED = 1;
        public static final double ALGAE_ANGLE_SPEED_MAGNIFIER = 1; // 0 - 1, directly multiplied to output power

        public static final double ALGAE_MOVE_SPEED = 1;
    }
}
