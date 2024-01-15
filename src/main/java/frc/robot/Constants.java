// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int MACROPAD_PORT = 2;
    }

    public static final class VisionConstants {
        public static final double GOAL_RANGE_METERS = 5;
    }

    //     /* SDS MK4 Potential Drive Gear Ratios*/
    //     /** SDS MK4 - 8.14 : 1 */
    //     public static final double SDSMK4_L1 = (8.14 / 1.0);
    //     /** SDS MK4 - 6.75 : 1 */
    //     public static final double SDSMK4_L2 = (6.75 / 1.0);
    //     /** SDS MK4 - 6.12 : 1 */
    //     public static final double SDSMK4_L3 = (6.12 / 1.0);
    //     /** SDS MK4 - 5.14 : 1 */
    //     public static final double SDSMK4_L4 = (5.14 / 1.0);


    /* Swerve drive constants (for SW Testbase) */
    public static final class Swerve {
        
        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21); // NOTE: Robot-specific
        public static final double WHEEL_BASE = Units.inchesToMeters(24); // NOTE: Robot-specific
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.4; //TODO: could be calculated/tuned
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final double MIN_THROTTLE_LEVEL = 0.2; //max robot power/speed when throttle not engaged

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* SDS Swerve Module Constants */
        public static final boolean DRIVE_MOTOR_INVERT = true;  /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean CANCODER_INVERT = false;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);/** 12.8 : 1 */ 
        public static final double DRIVE_GEAR_RATIO = (5.14 / 1.0);// for SDSMK4_L4

        /* Velocity and Position Scaling factors for built-in REV NEO encoders */
        public static final double DRIVE_ENCODER_ROT2METERS =  Math.PI * WHEEL_DIAMETER / DRIVE_GEAR_RATIO;
        public static final double DRIVE_ENCODER_RPM2METERSPERSEC = DRIVE_ENCODER_ROT2METERS / 60.0;

        /* NavX/gyro specific constants */
        public static final boolean GYRO_INVERT = false; // Always ensure Gyro is CCW+ CW-

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 19;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 12;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((-4.85 * Math.PI) / 4);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 20;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 15;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians(1.13 * Math.PI);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 9;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((-3.45 * Math.PI) / 4);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final int CANCODER_ID = 10;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((0.75 * Math.PI) / 2);
        }
    }
}