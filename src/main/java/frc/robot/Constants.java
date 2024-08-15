// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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
    public static class FieldConstants{
        public static final Pose2d RED_SPEAKER = new Pose2d(16.579342, 5.547868, new Rotation2d());
        public static final Pose2d RED_AMP = new Pose2d(14.700758, 8.2042, new Rotation2d());
        public static final Pose2d RED_SOURCE_RIGHT = new Pose2d(.356108,.883666, new Rotation2d());
        public static final Pose2d RED_SOURCE_LEFT = new Pose2d(1.461516,245872, new Rotation2d());

        public static final Pose2d BLUE_SPEAKER = new Pose2d(0, 5.547868, new Rotation2d());
        public static final Pose2d BLUE_AMP = new Pose2d(1.8415,8.2042, new Rotation2d());
        //public static final Pose2d BLUE_AMP = new Pose2d(1.82753,8.2042, new Rotation2d());
        public static final Pose2d BLUE_SOURCE_RIGHT = new Pose2d(15.079472, .245872, new Rotation2d());
        public static final Pose2d BLUE_SOURCE_LEFT = new Pose2d(16.185134, .883666, new Rotation2d());

    //    public static final double SPEAKER_HEIGHT_METERS = 1.576 - (7*0.0254); 
        public static final double SPEAKER_HEIGHT_METERS = 1.576 - (5*0.0254); 
       //public static final double SPEAKER_HEIGHT_METERS = 1.576; 
    }

    public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class VisionConstants {
        public static final double GOAL_RANGE_METERS = 5;
        public static final double APRIL_TAG_SD_X = 1.0; //base vision std dev values for pose estimator 
        public static final double APRIL_TAG_SD_Y = 1.0; 
        public static final double APRIL_TAG_CUTOFF_DISTANCE = 4.0; //Distance (in meters) beyond which no april tag is folded in with Odometry
    }
    public static final class IntakeConstants{
        public static final int INTAKE_MOTOR_ID = 10; 
    }

    public static final class FeederConstants{
        public static final int FEEDER_MOTOR_ID = 12;
        public static final double FEEDER_MOTOR_POWER = -0.7;
        public static enum FeederState {
            READYINPUT,
            INTAKE,
            TRANSIT,
            READYSHOOT,
        }
        public static final int INTAKE_BEAMBREAK_ID = 1;
        public static final int SHOOTER_BEAMBREAK_ID = 2; 
    }

    public static final class ArmConstants{
        public static final int ARM_MOTOR_ID_LEFT = 13;
        public static final int ARM_MOTOR_ID_RIGHT = 16; 
        public static final double ARM_MOTOR_GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (4.0 / 1.0) *(58.0 / 22.0); // two motor gear boxes + extra gearing
        public static final double ARM_RESTING_POSITION_ANGLE = 23.0; 
        public static final double ARM_AMP_ANGLE = 45.0;
        public static final double ARM_ENCODER_SCALING_FACTOR = 360.0 / ARM_MOTOR_GEAR_RATIO; //360 degrees/(gear ratio) 
        public static final double ARM_PROP_ANGLE = 55.187355041503906; 
        public static final double ARM_BACK_ANGLE = 126;
        public static final double ARM_STRAIGHT_UP = 90;
        public static final double ARM_OFF_PROP = 55.187355041503906 + 26; 
    }

    public static final class ShooterConstants{
        public static final int SHOOTER_MOTOR_ID_LEAD = 14;
        public static final int SHOOTER_MOTOR_ID_FOLLOW = 18; 
    }

    public static final class ClimberConstants{
        public static final int CLIMBER_MOTOR_ID = 15;
        public static final int CLIMBER_BEAMBREAK_ID = 3;

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
        public static final double TRACK_WIDTH = Units.inchesToMeters(18); // NOTE: Robot-specific
        public static final double WHEEL_BASE = Units.inchesToMeters(20); // NOTE: Robot-specific
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.4; //TODO: could be calculated/tuned
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final double MIN_THROTTLE_LEVEL = 0.2; //max robot power/speed when throttle not engaged

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), //fl
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), //fr
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), //bl
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)); //br

        /* SDS Swerve Module Constants */
        public static final boolean DRIVE_MOTOR_INVERT = false;  /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean CANCODER_INVERT = false;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);/** 12.8 : 1 */ 
        public static final double DRIVE_GEAR_RATIO = (5.14 / 1.0);// for SDSMK4_L4

        /* Velocity and Position Scaling factors for built-in REV NEO encoders */
        public static final double DRIVE_ENCODER_ROT2METERS =  Math.PI * WHEEL_DIAMETER / DRIVE_GEAR_RATIO;
        public static final double DRIVE_ENCODER_RPM2METERSPERSEC = DRIVE_ENCODER_ROT2METERS / 60.0;
        public static final double ANGLE_ENCODER_ROT2RAD = Math.PI * 2.0;
        

        /* NavX/gyro specific constants */
        public static final boolean GYRO_INVERT = false; // Always ensure Gyro is CCW+ CW-

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot 
            // public static final int DRIVE_MOTOR_ID = 7;
            // public static final int ANGLE_MOTOR_ID = 8;
            // public static final int CANCODER_ID = 21;

            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((-4.85 * Math.PI) / 4);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            // public static final int DRIVE_MOTOR_ID = 5;
            // public static final int ANGLE_MOTOR_ID = 6;
            // public static final int CANCODER_ID = 22;

            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians(1.13 * Math.PI);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot


            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((-3.45 * Math.PI) / 4);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            // public static final int DRIVE_MOTOR_ID = 2;
            // public static final int ANGLE_MOTOR_ID = 11;
            // public static final int CANCODER_ID = 24;

            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 21;
            
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRadians((0.75 * Math.PI) / 2);
        }
    }
}
