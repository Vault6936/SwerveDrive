// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class CANIds {
        public static final SwerveCANId leftFront = new SwerveCANId(2, 3, 21); //module 1
        public static final SwerveCANId rightFront = new SwerveCANId(5, 4, 22); //module 2
        public static final SwerveCANId leftBack = new SwerveCANId(7, 5, 23); //module 3
        public static final SwerveCANId rightBack = new SwerveCANId(10, 6, 24); //module 4
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
        public static final double driveMultiplier = 0.5; //Random constant.  Will it work?
        public static final double rotMultiplier = 0.5;
    }
}
