// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    m_climberMotor.setSmartCurrentLimit(30); 
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
    // This method will be called once per scheduler run
  }
}
