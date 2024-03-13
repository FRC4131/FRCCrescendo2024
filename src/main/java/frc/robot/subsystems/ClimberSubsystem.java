// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_climberMotor;
  private RelativeEncoder m_encoder; 

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor = new CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_climberMotor.setSmartCurrentLimit(30); 
    m_climberMotor.setInverted(true);
    m_climberMotor.setIdleMode(IdleMode.kBrake); 
    m_encoder = m_climberMotor.getEncoder(); 
    m_encoder.setPosition(0.0);
  }

  public void setPower(double power)
  {
    m_climberMotor.set(power);
  }

  public Command setPowerCommand(double power){
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter encoder", m_encoder.getPosition()); 
    // This method will be called once per scheduler run
  }
}
