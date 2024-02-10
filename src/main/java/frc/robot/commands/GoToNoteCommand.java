// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//targets notes using Google Coral and LL3 Detection Pipeline -- rotates and drives towards notes 
public class GoToNoteCommand extends Command {
  private VisionSubsystem m_visionSubsystem; 
  private DrivetrainSubsystem m_DrivetrainSubsystem; 
  private IntakeSubsystem m_IntakeSubsystem; 
  private PIDController m_angleController; 
  private DoubleSupplier m_x; 
  private DoubleSupplier m_y; 
  private DoubleSupplier m_throttle; 
  private Boolean m_fieldRelative; 

  /** Creates a new GoToNoteCommand. */
  public GoToNoteCommand(DrivetrainSubsystem drivetrainSubsystem, 
    VisionSubsystem visionSubsystem, 
    IntakeSubsystem intakeSubsystem, 
    DoubleSupplier x, 
    DoubleSupplier y, 
    DoubleSupplier throttle, 
    Boolean fieldRelative) {
    m_visionSubsystem = visionSubsystem; 
    m_DrivetrainSubsystem = drivetrainSubsystem; 
    m_IntakeSubsystem = intakeSubsystem; 
    m_x = x; 
    m_y = y; 
    m_throttle = throttle; 
    m_angleController = new PIDController(6.0, 0, 0); 
    m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    m_fieldRelative = fieldRelative; 
    addRequirements(m_DrivetrainSubsystem, m_visionSubsystem, m_IntakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angleController.reset(); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Double rotOutput = 0.0;
      Optional<Double> noteTx = m_visionSubsystem.getNoteOffset();  //gets horiz distance between note cross hair and LL3 crosshair 
      if (noteTx.isPresent()) //if the robot sees a note
      {
          m_angleController.setSetpoint(0.0); //goal: tx == 0
          rotOutput = m_angleController.calculate(noteTx.get() * (Math.PI / 180)); //gets tx and converts to radians 
      }

      double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; //controls throttle 
      double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL; 

      m_DrivetrainSubsystem.drive(new Translation2d(-0.8 * scale,
        m_y.getAsDouble() * scale),
        rotOutput,
        new Rotation2d(),
        m_fieldRelative,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
