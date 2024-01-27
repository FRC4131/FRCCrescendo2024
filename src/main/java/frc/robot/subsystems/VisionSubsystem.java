// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EstimatedRobotPose;



public class VisionSubsystem extends SubsystemBase {
  private Optional<EstimatedRobotPose> m_estimatedRobotPose;

  private int m_numDetections = 3;
  private int m_detectionThreshold = 9;
  private ArrayList<Boolean> m_tagDetections = new ArrayList<Boolean>();
  private NetworkTable m_NetworkTable; 
  private double m_magnitude; 
  private Vector m_targetVector; 

  public VisionSubsystem() {
    m_NetworkTable = NetworkTableInstance.getDefault().getTable("limelight-front"); 
    m_targetVector = VecBuilder.fill(20, 20, 20); //arbitrary big values
  }

  public Optional<EstimatedRobotPose> getAprilTagRobotPose() { //returns current april tag robot pose 
    return m_estimatedRobotPose;
  }

  public Optional<EstimatedRobotPose> aprilTagUpdate()
  {
    double rawBotPose[];
    Boolean validTargetsPresent = (1.0 == NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0));
    SmartDashboard.putBoolean("valid targets", validTargetsPresent);

    if (validTargetsPresent) //check to see if valid targets present 
    {
      rawBotPose = m_NetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]); //gets blue pose (one coordinate system)
      double[] targetArr = m_NetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[7]); 
      m_targetVector = VecBuilder.fill(targetArr[0], targetArr[1], targetArr[2]); 
      SmartDashboard.putNumber("pose x", rawBotPose[0]);
      SmartDashboard.putNumber("pose y", rawBotPose [1]);
      SmartDashboard.putNumber("pose z", rawBotPose [2]);

      return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d (rawBotPose[0], rawBotPose[1], rawBotPose[2]),
         new Rotation3d(Math.toRadians(rawBotPose[3]), 
          Math.toRadians(rawBotPose[4]), Math.toRadians(rawBotPose[5]))).toPose2d(),
          Timer.getFPGATimestamp() - (rawBotPose[6]/1000.0),
          m_targetVector)); //returns pose based on april tag
    }
    else{
      return Optional.empty(); 
    }
  }

  public boolean seesTargets()
  {
    return aprilTagUpdate().isPresent();
  }


  @Override
  public void periodic() {
    m_estimatedRobotPose = aprilTagUpdate();
    // SmartDashboard.putBoolean("IsAprilTagPersist", IsAprilTagPersistent());
    // SmartDashboard.putNumber("tv_test", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    // SmartDashboard.putNumberArray("Bot Pose", NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
    

    if (m_estimatedRobotPose.isPresent()) { //if optional contains a value 
      SmartDashboard.putNumber( "April Tag X", m_estimatedRobotPose.get().getPose().getX()); //returns robot x and y values + heading 
      SmartDashboard.putNumber("April Tag Y", m_estimatedRobotPose.get().getPose().getY());
      SmartDashboard.putNumber("April Tag Heading", m_estimatedRobotPose.get().getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("April Tag Mag", m_estimatedRobotPose.get().getMagnitude());
     }

  }

}