// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax m_shooterMotorLead; 
  private CANSparkMax m_shooterMotorFollow; 
  private RelativeEncoder m_shooterEncoder; 
  public double m_speed = 0; 
  public double m_targetRPM = -1100; 
  public boolean m_readyRumble; 

  public ShooterSubsystem() {
    m_shooterMotorLead = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID_LEAD, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotorLead.setSmartCurrentLimit(30); 
    m_shooterMotorLead.setIdleMode(IdleMode.kCoast);
    m_shooterMotorLead.setInverted(true);

    m_shooterMotorFollow = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID_FOLLOW, CANSparkLowLevel.MotorType.kBrushless);
   m_shooterMotorFollow.follow(m_shooterMotorLead, false); 
    m_shooterMotorFollow.setSmartCurrentLimit(30); 
    m_shooterMotorFollow.setIdleMode(IdleMode.kCoast); 
    //m_shooterMotorFollow.setInverted(true);
    

    m_readyRumble = false; 

    m_shooterEncoder = m_shooterMotorLead.getEncoder(); 

  }

  public void setPower(double power) {
  
      m_shooterMotorLead.set(power);
      DataLogManager.log("SHOOTER POWER: " + power);
  }


  public boolean isSpunUp()
  {
    return m_readyRumble; 
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  //   public Command setPowerCommand(double power) {
  //   return setDownPowerCommand(power).andThen(setUpPowerCommand(power)); 
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shooterEncoder.getVelocity());
    if (m_shooterEncoder.getVelocity() > 5100)
    {
      m_readyRumble = true; 
    }
    else{
      m_readyRumble = false; 
    }
    SmartDashboard.putBoolean("Rumble ready", m_readyRumble); 
  }
}
