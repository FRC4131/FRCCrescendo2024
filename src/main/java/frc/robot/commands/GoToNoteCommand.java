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
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GoToNoteCommand extends Command {
  private VisionSubsystem m_visionSubsystem; 
  private DrivetrainSubsystem m_DrivetrainSubsystem; 
  private PoseEstimationSubsystem m_PoseEstimationSubsystem; 
  private IntakeSubsystem m_IntakeSubsystem; 
  private PIDController m_angleController; 
  private DoubleSupplier m_x; 
  private DoubleSupplier m_y; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative; 
  /** Creates a new GrabNoteCommand. */
  public GoToNoteCommand(DrivetrainSubsystem drivetrainSubsystem, 
    VisionSubsystem visionSubsystem, 
    IntakeSubsystem intakeSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem, 
    DoubleSupplier x, 
    DoubleSupplier y, 
    DoubleSupplier throttle, 
    Boolean fieldRelative) {
    m_visionSubsystem = visionSubsystem; 
    m_DrivetrainSubsystem = drivetrainSubsystem; 
    m_IntakeSubsystem = intakeSubsystem; 
    m_PoseEstimationSubsystem = poseEstimationSubsystem; 
    m_x = x; 
    m_y = y; 
    m_throttle = throttle; 
    m_angleController = new PIDController(4, 0, 0);
    m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    m_fieldRelative = fieldRelative; 
    addRequirements(m_DrivetrainSubsystem, m_visionSubsystem, m_IntakeSubsystem, m_PoseEstimationSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angleController.reset(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Pose2d robotPose = m_PoseEstimationSubsystem.getPose(); 
      Optional<Double> noteTx = m_visionSubsystem.getNoteOffset(); 
      Double desiredRotRadians = 0.0; 
      if (noteTx.isPresent())
      {
        desiredRotRadians = noteTx.get() * (Math.PI / 180); 
      }
      m_angleController.setSetpoint(desiredRotRadians);
      m_angleController.calculate(robotPose.getRotation().getRadians()); 

      double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
      double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 

      m_DrivetrainSubsystem.drive(new Translation2d(m_x.getAsDouble() * scale,
        m_y.getAsDouble() * scale),
        desiredRotRadians,
        m_PoseEstimationSubsystem.getPose().getRotation(),
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
