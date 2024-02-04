// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_intakeController = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  private SparkPIDController m_intakePID;

  public IntakeSubsystem() {
    m_encoder = m_intakeController.getEncoder();
    m_intakeController.setSmartCurrentLimit(30, 40);

    m_intakePID =  m_intakeController.getPIDController(); 
    m_intakeController.burnFlash();

    m_intakePID.setP(1);
  }

  public void intakeSpeed(double d) {
    m_intakePID.setI(d * 1);
  }

  public void setIntakePower(double power) {
    m_intakeController.set(power);
  }

  public Command setIntakePowerCommand(double power) {
    return new InstantCommand(() -> {
      setIntakePower(power);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
