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
 * numerical or boolean
 * constants    INTAKEBACK(-117,0), 
        INTAKEFRONT(119, 5.25),
        INTAKEFRONTTELEOP(116, 11),
        SALUTE(90,0),
        AUTONCUBEHIGH(-57, 5),
        AUTONCUBECOMMIT(-57, 12),
        ACK(113, 11.75);. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kMacropadPort = 2;
    }

    public static final class VisionConstants {
        public static final double GOAL_RANGE_METERS = 5;
    }

    public class SDSMK4_Constants {

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = false;
        public static final boolean canCoderInvert = false;
        public static final double wheelDiameter = Units.inchesToMeters(4.0);

        public static final double angleGearRatio = (12.8 / 1.0);/** 12.8 : 1 */ 
        public static final double driveGearRatio = (5.14 / 1.0);// for SDSMK4_L4

        // public class driveGearRatios {
        //     /* SDS MK4 */
        //     /** SDS MK4 - 8.14 : 1 */
        //     public static final double SDSMK4_L1 = (8.14 / 1.0);
        //     /** SDS MK4 - 6.75 : 1 */
        //     public static final double SDSMK4_L2 = (6.75 / 1.0);
        //     /** SDS MK4 - 6.12 : 1 */
        //     public static final double SDSMK4_L3 = (6.12 / 1.0);
        //     /** SDS MK4 - 5.14 : 1 */
        //     public static final double SDSMK4_L4 = (5.14 / 1.0);
        // }
    }
        

    public static final class Swerve {
        // 1.15
            /* Drive Gear Ratios for all supported modules */

        public static final double kDriveEncoderRot2Meter =  Math.PI * SDSMK4_Constants.wheelDiameter / SDSMK4_Constants.driveGearRatio;
        public static final double kTurningEncoderRot2Rad = 2 * Math.PI / SDSMK4_Constants.angleGearRatio;
        public static final double kDriveEncoderRPM2MeterPerSec = Math.PI * (SDSMK4_Constants.wheelDiameter) / (SDSMK4_Constants.driveGearRatio * 60);//kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants IN METERS */
        public static final double trackWidth = Units.inchesToMeters(21); // TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24); // TODO: This must be tuned to specific robot
        //public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 120.0 *
                // COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Fast *
                // wheelCircumference;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.4;
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 4.4 /
                Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((-4.85 * Math.PI) / 4);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(1.13 * Math.PI);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((-3.45 * Math.PI) / 4);
          
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((0.75 * Math.PI) / 2);
            
        }
    }
}