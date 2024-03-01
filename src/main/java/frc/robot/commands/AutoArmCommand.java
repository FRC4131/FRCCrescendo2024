// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmCommand extends Command {
  private ArmSubsystem m_ArmSubsystem; 
  private double m_setpoint; 
  /** Creates a new AutoArmCommand. */
  public AutoArmCommand(ArmSubsystem armSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_setpoint = setpoint; 
    m_ArmSubsystem = armSubsystem; 
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.goToAngle(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ArmSubsystem.getArmAngle() > m_setpoint+3 || m_ArmSubsystem.getArmAngle() < m_setpoint-3)
    {
      return true; 
    }
    else
    {
      return false; 
    }
  }
}
