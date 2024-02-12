// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
      m_intakeMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
