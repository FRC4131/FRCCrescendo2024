// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonShootCommand extends Command {
  /** Creates a new AutonShootCommand. */
  private ShooterSubsystem m_shooterSubsystem;
  private FeederSubsystem m_feederSubsystem; 
  private double m_speed; 

  public AutonShootCommand(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSubsystem = feederSubsystem; 
    m_shooterSubsystem = shooterSubsystem; 
    m_speed = speed; 
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_shooterSubsystem.setPower(m_speed); 
      new WaitCommand(1.5); 
      m_feederSubsystem.setPower(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSubsystem.setFeederPowerCommand(0.0); 
    m_shooterSubsystem.setPowerCommand(0.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
