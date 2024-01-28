// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax m_BeltMotor;
  private DigitalInput m_SecondBreaker;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
      m_BeltMotor = new CANSparkMax(2, MotorType.kBrushless);
      m_SecondBreaker = new DigitalInput(1);
  }

  public void setPower(double power)
  {
    m_BeltMotor.set(power);
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  public boolean getSecondBreaker() {
    return m_SecondBreaker.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
