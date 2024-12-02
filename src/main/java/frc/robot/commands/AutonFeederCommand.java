// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonFeederCommand extends Command {
  private FeederSubsystem m_feederSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  //private Timer m_timer;
  
  /** Creates a new AutonFeederCommand. */
  public AutonFeederCommand(FeederSubsystem feederSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    //m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feederSubsystem.setPower(0.5);
    m_intakeSubsystem.setPower(1.0);
    // until(m_feederSubsystem.getShooterBreaker()){

    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_timer.stop();
    DataLogManager.log("End Feeder");
     m_feederSubsystem.setPower(0);
     m_intakeSubsystem.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_visionSubsystem.getSawNote() == false){
      return true;
    }
    if(!m_feederSubsystem.getShooterBreaker()){
      DataLogManager.log("Feeder 0.5 Seconds Passed");
      return true;  
    } else {
      return false;
    }
  }
}
