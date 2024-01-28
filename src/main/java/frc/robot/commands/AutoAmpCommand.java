// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AutoAmpCommand extends Command {
  DrivetrainSubsystem m_drivetrainSubsystem;
  PoseEstimationSubsystem m_poseEstimationSubsystem;
  Double m_desiredAngle;
  private DoubleSupplier m_controllerX; 
  private DoubleSupplier m_controllerY; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative;
  private Pose2d m_robotPose; 
  private Pose2d m_targetPose; 

  PIDController m_pidController;

  /** Creates a new FixateOnThing. */
  public AutoAmpCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem, 
    double angle,
    DoubleSupplier x,
    DoubleSupplier y, 
    DoubleSupplier throttle,
    Boolean fieldRelative, 
    Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_desiredAngle = angle;
    m_targetPose = targetPose;

    m_pidController = new PIDController(4, 0, 0);
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);

    m_controllerX = x; 
    m_controllerY = y;
    m_throttle = throttle; 
    m_fieldRelative = fieldRelative; 
    m_robotPose = m_poseEstimationSubsystem.getPose();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotPose = m_poseEstimationSubsystem.getPose();
    // Calculate the angle to always face the AprilTag
    m_desiredAngle = Math.atan2(m_targetPose.getY() - m_robotPose.getY(), m_targetPose.getX() - m_robotPose.getX());
    m_pidController.setSetpoint(m_desiredAngle);
    Double desiredRotation = m_pidController.calculate(m_poseEstimationSubsystem.getPose().getRotation().getRadians());

    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
    double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 
    m_drivetrainSubsystem.drive(new Translation2d(0, m_controllerY.getAsDouble() * scale),
        desiredRotation,
        m_poseEstimationSubsystem.getPose().getRotation(),
        m_fieldRelative,
        true);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
