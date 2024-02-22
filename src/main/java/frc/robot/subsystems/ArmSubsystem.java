// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new SwivelArmSubsystem. */
  private CANSparkMax m_armMotor;  
  private RelativeEncoder m_armEncoder; 
  private PIDController m_armPidController; 
  private ArmFeedforward m_armFeedforward; 
  private double m_angleSetpoint = 0.0;

  public ArmSubsystem() {
    m_armMotor = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    m_armEncoder = m_armMotor.getEncoder(); 
    m_armPidController = new PIDController(0.01, 0, 0);
    m_armFeedforward = new ArmFeedforward(0, 0, 0, 0); 

    m_armMotor.setSmartCurrentLimit(30); 
    m_armMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setInverted(true);
    m_armEncoder.setPositionConversionFactor(Constants.ArmConstants.ARM_ENCODER_SCALING_FACTOR);
    m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
  }

  public void setPower(double power) {
    m_armMotor.set(power);
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  public Command rotateToAngleCommand(double angle) {
    return runOnce(() -> {
      goToAngle(angle);
    });
  }

  private void goToAngle(double angleDegrees)
  {
    m_angleSetpoint = angleDegrees; 
  }

  public Command goToAmpAngleCommand()
  {
    return runOnce(() -> {
      goToAngle(ArmConstants.ARM_AMP_ANGLE);
    }); 
  }

  public Command resetArmEncoderCommand()
  {
    return new InstantCommand(() ->
    {
      m_armEncoder.setPosition(0.0);
    }); 
  }

  public Command goToRestAngleCommand()
  {
    return runOnce(() -> {
      goToAngle(ArmConstants.ARM_RESTING_POSITION_ANGLE);
    }); 
  }

  public Command manualArmCommand(double power)
  {
    return new InstantCommand(() ->
    {
      m_armMotor.set(power);
    }); 
  }


  public void resetPosition() { //sends arm to 0 position and resets encoder to 0 
    this.goToRestAngleCommand(); 
    m_armEncoder.setPosition(0);
  }

  public double getArmAngle()
  {
    return m_armEncoder.getPosition(); 
  }

  @Override
  public void periodic() {
    double rawPower = m_armPidController.calculate(getArmAngle(), m_angleSetpoint);
    double clampedPower = MathUtil.clamp(rawPower, -0.1, 0.7);

    m_armMotor.set(clampedPower);
      
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Degrees", getArmAngle()); 
    SmartDashboard.putNumber("ArmSetpoint", m_angleSetpoint);
    SmartDashboard.putNumber("RawPower", rawPower);
    SmartDashboard.putNumber("ClampedPower", clampedPower);
    SmartDashboard.putData("ARM PID",m_armPidController);
  }
}
