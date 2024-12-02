// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Timer m_timer; 
  private Timer m_timer2;
  private boolean seesNote;
  private int restart;

  private PIDController m_angleController;

  /** Creates a new GoToNoteCommand. */
  public AutonGoToNoteCommand(DrivetrainSubsystem drivetrainSubsystem,
      VisionSubsystem visionSubsystem,
      IntakeSubsystem intakeSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_intakeSubsystem = intakeSubsystem; 
    m_timer = new Timer();
    m_timer2 = new Timer();
    
    m_angleController = new PIDController(5.0, 0, 0);
    m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_DrivetrainSubsystem, m_visionSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angleController.reset();
    m_timer.restart();
    DataLogManager.log("Auton Go To Note START");
    seesNote = false;
    m_visionSubsystem.setSawNote(false);
    restart = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DataLogManager.log("Auton Go To Note EXECUTE");
    Double rotOutput = 0.0;
    Optional<Double> noteTx = m_visionSubsystem.getNoteOffset(); // gets horiz distance between note cross hair and LL3
                                                                 // crosshair
    //seesNote = false;
    if (noteTx.isPresent()) // if the robot sees a note
    {
      m_angleController.setSetpoint(0.0); // goal: tx == 0
      rotOutput = m_angleController.calculate(noteTx.get() * -(Math.PI / 180)); // gets tx and converts to radians
      seesNote = true;
      m_visionSubsystem.setSawNote(true);
    }

    double vel_x = -Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 0.2;

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
        false,
        true);

    if (!m_visionSubsystem.seesNote()){ //If doesn't see note
      if(restart<2) // Only restarts timer once
      {
        restart++;
        restartTimer();
        DataLogManager.log("Timer Restart");
      }   
      //SmartDashboard.putNumber("restart", restart);
       if(m_timer.hasElapsed(0.25)){ //After 0.25 seconds stops the command
        DataLogManager.log("Reached 0.25 seconds");
        m_DrivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, new Rotation2d(), false, true);
     
       }
        if(m_timer.hasElapsed(0.25)){
           seesNote = false;
        }
    } 
    SmartDashboard.putNumber("seconds", m_timer.get());
      // if (m_timer.hasElapsed(3.0)){
      //   m_DrivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, new Rotation2d(), false, true);
      // }

      // if (m_timer.hasElapsed(5.0)){
      //   seesNote = false;
      // }

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        m_DrivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, new Rotation2d(), false, true);
          //DataLogManager.log("AG2N 2 Second Elapse");
          m_intakeSubsystem.setPower(0.0);
          DataLogManager.log("Intake STOP AG2N");
          m_timer2.stop();
          DataLogManager.log("Auton Go To Note END");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (seesNote == false)
    {
      DataLogManager.log("AG2N: doesn't see note");
      return true; 
    }
    else if (m_timer.hasElapsed(2))
    {
      DataLogManager.log("AG2N: TIMEOUT 2 SEC");
      return true; 
    }
    else{
      DataLogManager.log("AG2N: sees note");
       return false;
    }
  }
  public void restartTimer(){
    if(restart == 1){
      m_timer.restart();
    }
  }


}
