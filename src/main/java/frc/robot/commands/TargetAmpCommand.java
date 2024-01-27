// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class TargetAmpCommand extends Command {
  /** Creates a new TargetAmpCommand. */
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
  public TargetAmpCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem, 
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    Boolean m_fieldRelative,
    Pose2d targetPose) {

    m_drivetrainSubsystem = drivetrainSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
