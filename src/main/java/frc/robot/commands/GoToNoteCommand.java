// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//targets notes using Google Coral and LL3 Detection Pipeline -- rotates and drives towards notes 
public class GoToNoteCommand extends Command {
  private VisionSubsystem m_visionSubsystem;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  //private FeederSubsystem m_FeederSubsystem; 
  private PIDController m_angleController;
  private DoubleSupplier m_xSupplier; 
  private DoubleSupplier m_ySupplier;
  private DoubleSupplier m_throttle;

  private Boolean m_fieldRelative;

  /** Creates a new GoToNoteCommand. */
  public GoToNoteCommand(DrivetrainSubsystem drivetrainSubsystem,
      VisionSubsystem visionSubsystem,
      IntakeSubsystem intakeSubsystem,
      DoubleSupplier xSupplier, 
      DoubleSupplier ySupplier,
      DoubleSupplier throttle,
      Boolean fieldRelative) {
    m_visionSubsystem = visionSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_throttle = throttle;
    m_angleController = new PIDController(3.0, 0, 0);
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
    Optional<Double> noteTx = m_visionSubsystem.getNoteOffset(); // gets horiz distance between note cross hair and LL3
                                                                 // crosshair
    if (noteTx.isPresent()) // if the robot sees a note
    {
      m_angleController.setSetpoint(0.0); // goal: tx == 0
      rotOutput = m_angleController.calculate(noteTx.get() * (Math.PI / 180)); // gets tx and converts to radians
    }

    double slope = 1 - Constants.Swerve.MIN_THROTTLE_LEVEL; // Throttle control
    double scale = slope * m_throttle.getAsDouble() + Constants.Swerve.MIN_THROTTLE_LEVEL;
    double vel_x = -Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.4;
    m_IntakeSubsystem.setPower(1.0); 

    //Robot is driven (in Robot-Centric frame) towards note
    m_DrivetrainSubsystem.drive(new Translation2d(vel_x * scale,
        0.0),
        rotOutput,
        new Rotation2d(),
        false,
        true);

    // m_DrivetrainSubsystem.drive(new Translation2d(m_xSupplier.getAsDouble() * scale,
    // m_ySupplier.getAsDouble() * scale),
    //     rotOutput,
    //     new Rotation2d(),
    //     m_fieldRelative,
    //     true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
    m_IntakeSubsystem.setPower(0.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       return false;
  }
}
