// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.EstimatedRobotPose;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StdDevEstimatorCommand extends Command { //not currently used : test command used when trying to figure out variable std dev values for vision
  /** Creates a new StdDevEstimatorCommand. */
  private VisionSubsystem m_visionSubsystem; 
  private double[] m_xArray = new double [45]; 
  private double[] m_yArray = new double [45]; 
  private double[] m_thetaArr = new double [45]; 
  private Double m_sdX; 
  private Double m_sdY; 
  private Double m_sdTheta; 
  private double m_currentTimeStamp; 
  private int m_index = 0;

  public StdDevEstimatorCommand(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_visionSubsystem = visionSubsystem; 
    addRequirements();

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentTimeStamp = 0.0; 
    m_index = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> robotPose = m_visionSubsystem.getAprilTagRobotPose(); 
    if (robotPose.isPresent())
    {
      if (robotPose.get().getTimeStamp() > m_currentTimeStamp)
      {
          double rx = robotPose.get().getPose().getX(); 
          double ry = robotPose.get().getPose().getY(); 
          double rTheta = robotPose.get().getPose().getRotation().getRadians(); 

          m_xArray[m_index] = rx;
          m_yArray[m_index] = ry;
          m_thetaArr[m_index] = rTheta; 

          m_currentTimeStamp = robotPose.get().getTimeStamp(); 
      }


    }

    double meanX = 0;
    double meanY = 0;
    double meanTheta = 0; 
    int size = m_xArray.length; 

    for (int i = 0; i < size; i++)
    {
      meanX+=m_xArray[i];
      meanY+=m_yArray[i];
      meanTheta +=m_thetaArr[i];
    }
    meanX = meanX / size; 
    meanY = meanY / size; 
    meanTheta = meanTheta / size; 

    m_sdX = 0.0; 
    m_sdY = 0.0; 
    m_sdTheta = 0.0; 

    for (int i = 0; i < size; i++)
    {
      m_sdX += Math.pow(m_xArray[i] - meanX, 2);  
      m_sdY += Math.pow(m_yArray[i] - meanY, 2);  
      m_sdTheta += Math.pow(m_thetaArr[i] - meanTheta, 2);  
    }
    
    m_sdX = Math.sqrt(m_sdX / size);
    m_sdY = Math.sqrt(m_sdY / size);
    m_sdTheta = Math.sqrt(m_sdTheta / size);

    m_index++; 
    m_index = m_index % size; 

    SmartDashboard.putNumber("Sd x", m_sdX);
    SmartDashboard.putNumber("Sd y", m_sdY);
    SmartDashboard.putNumber("Sd theta", m_sdTheta);
    SmartDashboard.putNumber("SdIndex", m_index);  



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // double meanX = 0;
    // double meanY = 0;
    // double meanTheta = 0; 
    // double size = m_xArray.size(); 
    // for (int i = 0; i < size; i++)
    // {
    //   meanX+=m_xArray.get(i);
    //   meanY+=m_yArray.get(i);
    //   meanTheta += m_thetaArr.get(i);
    // }
    // meanX = meanX / size; 
    // meanY = meanY / size; 
    // meanTheta = meanTheta / size; 

    // m_sdX = 0.0; 
    // m_sdY = 0.0; 
    // m_sdTheta = 0.0; 

    // for (int i = 0; i < size; i++)
    // {
    //   m_sdX += Math.pow(m_xArray.get(i) - meanX, 2);  
    //   m_sdY += Math.pow(m_yArray.get(i) - meanY, 2);  
    //   m_sdTheta += Math.pow(m_thetaArr.get(i) - meanTheta, 2);  
    // }
    
    // m_sdX = Math.sqrt(m_sdX / size);
    // m_sdY = Math.sqrt(m_sdY / size);
    // m_sdTheta = Math.sqrt(m_sdTheta / size);

    // SmartDashboard.putNumber("Sd x", m_sdX);
    // SmartDashboard.putNumber("Sd y", m_sdY);
    // SmartDashboard.putNumber("Sd theta", m_sdTheta);  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
