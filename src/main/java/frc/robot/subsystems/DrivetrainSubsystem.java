// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;

    public frc.lib.util.SwerveModule[] m_SwerveMods;

    public DrivetrainSubsystem() {

        m_SwerveMods = new SwerveModule[] {
                new SwerveModule(0,
                    Constants.Swerve.Mod0.ANGLE_MOTOR_ID,
                    Constants.Swerve.Mod0.DRIVE_MOTOR_ID,
                    Constants.Swerve.Mod0.CANCODER_ID),
                new SwerveModule(1, 
                    Constants.Swerve.Mod1.ANGLE_MOTOR_ID,
                    Constants.Swerve.Mod1.DRIVE_MOTOR_ID,
                    Constants.Swerve.Mod1.CANCODER_ID),
                new SwerveModule(2,
                    Constants.Swerve.Mod2.ANGLE_MOTOR_ID,
                    Constants.Swerve.Mod2.DRIVE_MOTOR_ID,
                    Constants.Swerve.Mod2.CANCODER_ID),
                new SwerveModule(3,
                    Constants.Swerve.Mod3.ANGLE_MOTOR_ID,
                    Constants.Swerve.Mod3.DRIVE_MOTOR_ID,
                    Constants.Swerve.Mod3.DRIVE_MOTOR_ID)
        };
        for (SwerveModule mod : m_SwerveMods) {
            mod.reset();
        }
    }

    /**
     * 
     * @param translation     X and Y values (multiply by max speed)
     * @param rotation        Theta value (multiply by max angular velocity)
     * @param currentRotation The current rotation/theta of the robot (i.e. the yaw
     *                        from the gyro)
     * @param fieldRelative   Is field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, Rotation2d currentRotation, boolean fieldRelative,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        currentRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates);
    }

    // public double getYaw() {
    //     return (Constants.Swerve.invertGyro) ? (360 - m_navX.getYaw())
    //             : (m_navX.getYaw());
    // }

    // public void zeroGyro(){
    //     m_navX.reset();
    // }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Overloaded drive method.
     * @param chassisSpeeds to drive at.
     * @param speedCapTranslation in m/s.
     * @param speedCapRotation in rad/s.
     */
    public void drive(ChassisSpeeds chassisSpeeds, double speedCapTranslation, double speedCapRotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, chassisSpeeds, speedCapTranslation,
                speedCapTranslation, speedCapRotation);
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_SwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_SwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        // for (SwerveModule mod : m_SwerveMods) {
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Position", mod.getDrivePosition());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Turn Position", mod.getTurningPosition());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }

    }
}