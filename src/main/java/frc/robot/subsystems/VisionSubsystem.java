// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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

public class VisionSubsystem extends SubsystemBase { // handles LL3 April Tag Detection
  private Optional<EstimatedRobotPose> m_estimatedRobotPose;

  private NetworkTable m_NetworkTableFront;
  private NetworkTable m_NetworkTableBack;
  private Vector m_targetVector; // holds vector from target to camera

  public VisionSubsystem() {
    m_NetworkTableFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    m_NetworkTableBack = NetworkTableInstance.getDefault().getTable("limelight-back");
    m_targetVector = VecBuilder.fill(20, 20, 20); // arbitrary big values -- if the robot starts out not seeing april
                                                  // tag, it does not trust from the get go
  }

  public Optional<EstimatedRobotPose> getAprilTagRobotPose() { // returns current april tag robot pose
    return m_estimatedRobotPose;
  }

  public boolean seesNote() {
    return (1.0 == m_NetworkTableBack.getEntry("tv").getDouble(0));
  }

  public boolean seesSpeakerTags() {
    DriverStation.refreshData();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean seesSpeaker = false;
    if (!alliance.isEmpty()) {
      if (alliance.get().equals(Alliance.Blue)) {
        seesSpeaker = (m_NetworkTableFront.getEntry("tid").getDouble(0) == 8
            || m_NetworkTableFront.getEntry("tid").getDouble(0) == 7);
      } else if (alliance.get().equals(Alliance.Red)) {
        seesSpeaker = (m_NetworkTableFront.getEntry("tid").getDouble(0) == 3
            || m_NetworkTableFront.getEntry("tid").getDouble(0) == 4);
      }
    }
    return seesSpeaker;
  }

  public boolean seesAmpTags() {
    DriverStation.refreshData();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean seesAmp = false;
    if (!alliance.isEmpty()) {
      if (alliance.get().equals(Alliance.Blue)) {
        seesAmp = (m_NetworkTableFront.getEntry("tid").getDouble(0) == 6);
      } else if (alliance.get().equals(Alliance.Red)) {
        seesAmp = (m_NetworkTableFront.getEntry("tid").getDouble(0) == 5);
      }
    }

    return seesAmp;
  }


  public Optional<Double> getNoteOffset() {
    if (seesNote()) {
      double tx = m_NetworkTableBack.getEntry("tx").getDouble(0);
      return Optional.of(tx);
    } else {
      return Optional.empty();
    }
  }

  public Optional<Double> getAmpOffset() {
    if (seesAmpTags()) {
      double tx = m_NetworkTableFront.getEntry("tx").getDouble(0);
      return Optional.of(tx);
    } else {
      return Optional.empty();
    }
  }

  public Optional<EstimatedRobotPose> aprilTagUpdate() // updates estimated robot pose based on april tags seen
  {
    double rawBotPose[];
    Boolean validTargetsPresent = (1.0 == m_NetworkTableFront.getEntry("tv").getDouble(0));
    // SmartDashboard.putBoolean("valid targets", validTargetsPresent);

    if (validTargetsPresent) // returns bot pose if april tags are seen
    {
      // gets blue pose (assuming a single coordinate system for both alliances)
      rawBotPose = m_NetworkTableFront.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
      // SmartDashboard.putNumber("pose x", rawBotPose[0]);
      // SmartDashboard.putNumber("pose y", rawBotPose [1]);
      // SmartDashboard.putNumber("pose z", rawBotPose [2]);

      double[] targetArr = m_NetworkTableFront.getEntry("targetpose_cameraspace").getDoubleArray(new double[7]); // gets
                                                                                                                 // vector
                                                                                                                 // from
                                                                                                                 // target
                                                                                                                 // to
                                                                                                                 // camera
      m_targetVector = VecBuilder.fill(targetArr[0], targetArr[1], targetArr[2]); // passes x, y, z values into 3d
                                                                                  // vector (so we can get l3 norm
                                                                                  // instead of l7)

      return Optional.of(new EstimatedRobotPose(
          new Pose3d(new Translation3d(rawBotPose[0], rawBotPose[1], rawBotPose[2]),
              new Rotation3d(Math.toRadians(rawBotPose[3]),
                  Math.toRadians(rawBotPose[4]), Math.toRadians(rawBotPose[5])))
              .toPose2d(),
          Timer.getFPGATimestamp() - (rawBotPose[6] / 1000.0),
          m_targetVector)); // returns pose based on april tag
    } else {
      return Optional.empty(); // returns an empty if not seeing april tags
    }
  }

  public boolean seesTargets() // returns whether robot sees april tags (used for triggers)
  {
    return aprilTagUpdate().isPresent();
  }

  @Override
  public void periodic() {
    m_estimatedRobotPose = aprilTagUpdate(); // constantly updates bot pose
    SmartDashboard.putBoolean("sees note", seesNote());
    SmartDashboard.putBoolean("sees amp", seesAmpTags());
    if (m_estimatedRobotPose.isPresent()) { // if optional contains a value
      SmartDashboard.putNumber("April Tag X", m_estimatedRobotPose.get().getPose().getX()); // returns robot x and y
                                                                                            // values + heading
      SmartDashboard.putNumber("April Tag Y", m_estimatedRobotPose.get().getPose().getY());
      SmartDashboard.putNumber("April Tag Heading", m_estimatedRobotPose.get().getPose().getRotation().getDegrees());
    }

  }

}
