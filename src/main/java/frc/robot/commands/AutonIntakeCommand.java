// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonIntakeCommand extends Command {
  /** Creates a new AutonIntakeCommand. */
  private IntakeSubsystem m_intakeSubsystem; 
  private FeederSubsystem m_feederSubsystem; 


  public AutonIntakeCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem; 
    m_intakeSubsystem = intakeSubsystem; 

    addRequirements(intakeSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (m_feederSubsystem.getShooterBreaker())
    {
      m_intakeSubsystem.setPowerCommand(0.7); 
      m_feederSubsystem.setFeederPowerCommand(0.5); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setPowerCommand(0.0); 
    m_feederSubsystem.setFeederPowerCommand(0.0); 

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
