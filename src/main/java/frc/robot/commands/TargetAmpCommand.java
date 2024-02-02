// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.EstimatedRobotPose;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetAmpCommand extends Command {
  /** Creates a new TargetAmpCommand. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private PoseEstimationSubsystem m_poseEstimationSubsystem;
  
  Double m_desiredAngle;
  private DoubleSupplier m_controllerX; 
  private DoubleSupplier m_controllerY; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative;
  private Pose2d m_robotPose; 
  private Pose2d m_targetPose; 

  PIDController m_pidController;
  public TargetAmpCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem,  
    DoubleSupplier ySupplier,
    DoubleSupplier throttle, 
    Boolean fieldRelative,
    Pose2d targetPose) {

      m_drivetrainSubsystem = drivetrainSubsystem;
      m_poseEstimationSubsystem = poseEstimationSubsystem;
      m_targetPose = targetPose;
  
      m_pidController = new PIDController(4, 0, 0);
      m_pidController.enableContinuousInput(-Math.PI, Math.PI);
      addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);
  
      m_controllerY = ySupplier;
      m_throttle = throttle; 
      m_fieldRelative = fieldRelative; 
      m_robotPose = m_poseEstimationSubsystem.getPose();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotPose = m_poseEstimationSubsystem.getPose();
    double robotX = m_robotPose.getX(); 
    double robotY = m_robotPose.getY(); 
    
    m_pidController.setSetpoint(m_desiredAngle);
    Double desiredRotation = 0.0;
    Double desiredX = m_pidController.calculate(Constants.FieldConstants.BLUE_AMP.getX()); 

    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
    double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 
    m_drivetrainSubsystem.drive(new Translation2d(desiredX,
        m_controllerY.getAsDouble() * scale),
        desiredRotation,
        m_poseEstimationSubsystem.getPose().getRotation(),
        m_fieldRelative,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
