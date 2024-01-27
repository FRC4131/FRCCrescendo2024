// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.opencv.core.Point;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EstimatedRobotPose;
import frc.robot.Constants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  VisionSubsystem m_visionSubsystem;
  private final Field2d field2d = new Field2d();
  static SwerveDrivePoseEstimator m_swerveDrivePoseEst;
  AHRS m_navX;
  
  private Double m_sdX; 
  private Double m_sdY; 
  private Double m_sdTheta; 

  public frc.lib.util.SwerveModule[] mSwerveMods;

  /** Creates a new PoseEstimationSubsystem. */
  public PoseEstimationSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem)
  {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    this.zeroGyro();

    m_swerveDrivePoseEst = new SwerveDrivePoseEstimator(
        Constants.Swerve.SWERVE_KINEMATICS,
        getGyroYaw(),
        m_drivetrainSubsystem.getModulePositions(),
        new Pose2d()
        ,VecBuilder.fill(0.1, 0.1, 0.1), //odometry 
        VecBuilder.fill(Constants.VisionConstants.APRIL_TAG_SD_X, Constants.VisionConstants.APRIL_TAG_SD_Y, 0.9) //april tags
        );
    
        
  }

  public void zeroGyro() {
    m_navX.zeroYaw();
  }

  public void zeroAngle(){
    m_navX.zeroYaw();
    m_swerveDrivePoseEst.resetPosition(getGyroYaw(), m_drivetrainSubsystem.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Command zeroAngleCommand(){
    return new InstantCommand(()->this.zeroAngle());
  }

  public void setAngleAdjustment(double adjustment){
    m_navX.setAngleAdjustment(adjustment);
  }

  private Rotation2d getGyroYaw() {
    return (Constants.Swerve.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - m_navX.getYaw())
        : Rotation2d.fromDegrees(m_navX.getYaw());
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveDrivePoseEst.resetPosition(getGyroYaw(), m_drivetrainSubsystem.getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return m_swerveDrivePoseEst.getEstimatedPosition();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public double getRoll() {
    return m_navX.getRoll();
  }

  public double getYaw() {
    return m_navX.getYaw();
  }

  public boolean isInRadius(Pose2d targetPose, double radialThreshold)
  { 
      double dx = Math.pow(targetPose.getX() - m_swerveDrivePoseEst.getEstimatedPosition().getX(), 2); 
      double dy = Math.pow(targetPose.getY() - m_swerveDrivePoseEst.getEstimatedPosition().getY(), 2);
      double radius = Math.sqrt(dx + dy); 
      return (radius < radialThreshold); 
  }

  @Override
  public void periodic() {
    EstimatedRobotPose aprilTagPose = m_visionSubsystem.getAprilTagRobotPose().orElse(null);
    DriverStation.refreshData(); 

    SmartDashboard.putBoolean("test", false);
    if (aprilTagPose != null && (!DriverStation.isAutonomous())) {
      SmartDashboard.putNumber("MAGNITUDE", aprilTagPose.getMagnitude());
      //updates std values based on magnitude of the vector from camera to april tag (trusts it less as we go back more)
      m_swerveDrivePoseEst.setVisionMeasurementStdDevs( 
        VecBuilder.fill(Constants.VisionConstants.APRIL_TAG_SD_X * aprilTagPose.getMagnitude(),
        Constants.VisionConstants.APRIL_TAG_SD_Y * aprilTagPose.getMagnitude(), 
        10));

      m_swerveDrivePoseEst.addVisionMeasurement(aprilTagPose.getPose(), aprilTagPose.getTimeStamp());
      SmartDashboard.putBoolean("test", true);
    }
    m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions());
    field2d.setRobotPose(m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions()));
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawGyroYaw", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("SwervePoseEst x", m_swerveDrivePoseEst.getEstimatedPosition().getX());
    SmartDashboard.putNumber("SwervePoseEst y", m_swerveDrivePoseEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odom Rotation", m_swerveDrivePoseEst.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Pitch", getPitch());
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putNumber("Robot Yaw", getYaw());
    
    SmartDashboard.putData(field2d);

  }

}
