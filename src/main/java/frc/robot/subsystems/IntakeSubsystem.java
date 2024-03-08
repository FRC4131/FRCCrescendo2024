// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;
  private RelativeEncoder m_intakeEncoder; 
  private boolean m_intaking; 

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
      m_intakeMotor = new CANSparkMax(Constants.FeederConstants.INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
      m_intakeMotor.setSmartCurrentLimit(30);
      m_intakeMotor.setInverted(true);
      m_intakeEncoder = m_intakeMotor.getEncoder();
      m_intaking = false; 
  }

  public void setPower(double power)
  {
    m_intakeMotor.set(power);
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  public boolean isIntaking()
  {
    return m_intaking; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_intakeEncoder.getVelocity() > 50)
    {
      m_intaking = true; 
    }
    else {
      m_intaking = false; 
    }
  }
}
