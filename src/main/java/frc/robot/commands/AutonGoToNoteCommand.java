// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//targets notes using Google Coral and LL3 Detection Pipeline -- rotates and drives towards notes 
public class AutonGoToNoteCommand extends Command {
  private VisionSubsystem m_visionSubsystem;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private IntakeSubsystem m_intakeSubsystem; 

  private PIDController m_angleController;

  /** Creates a new GoToNoteCommand. */
  public AutonGoToNoteCommand(DrivetrainSubsystem drivetrainSubsystem,
      VisionSubsystem visionSubsystem,
      IntakeSubsystem intakeSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_intakeSubsystem = intakeSubsystem; 
    
    m_angleController = new PIDController(5.0, 0, 0);
    m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_DrivetrainSubsystem, m_visionSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angleController.reset();
    DataLogManager.log("Auton Go To Note START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DataLogManager.log("Auton Go To Note EXECUTE");
    Double rotOutput = 0.0;
    Optional<Double> noteTx = m_visionSubsystem.getNoteOffset(); // gets horiz distance between note cross hair and LL3
                                                                 // crosshair
    if (noteTx.isPresent()) // if the robot sees a note
    {
      m_angleController.setSetpoint(0.0); // goal: tx == 0
      rotOutput = m_angleController.calculate(noteTx.get() * (Math.PI / 180)); // gets tx and converts to radians
    }

    double vel_x = -Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.3;

    //Robot is driven (in Robot-Centric frame) towards note
    // m_DrivetrainSubsystem.drive(new Translation2d(vel_x * scale,
    //     0.0),
    //     rotOutput,
    //     new Rotation2d(),
    //     m_fieldRelative,
    //     true);
    m_intakeSubsystem.setPower(1.0);
    m_DrivetrainSubsystem.drive(new Translation2d(vel_x, 
        0.0),
        rotOutput,
        new Rotation2d(),
        true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
    DataLogManager.log("Auton Go To Note END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_visionSubsystem.seesNote())
    {
      return true; 
    }
    else{
       return false;
    }
  }
}
