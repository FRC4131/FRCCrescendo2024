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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

// arm subsystem controls the pivot of the arm/indexing mechanism. uses 2 motors. in this iteration: left = leader, right = follower

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_armMotorL;  //lead/left motor
  private CANSparkMax m_armMotorR; //follower/right motor
  private RelativeEncoder m_armEncoder; 
  private PIDController m_armPidController; 
  private double m_speakerHeightOffset; 
  private double m_angleSetpoint = Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE;
  private DigitalInput m_frontLimit = new DigitalInput(0); 
  private DigitalInput m_backLimit = new DigitalInput(1); 
  private IntakeSubsystem m_IntakeSubsystem; 

  public ArmSubsystem(IntakeSubsystem intakeSubsystem) {

    m_IntakeSubsystem = intakeSubsystem; 

    //configures left motor (inverted) 
    m_armMotorL = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR_ID_LEFT, MotorType.kBrushless); 
    m_armMotorL.setSmartCurrentLimit(30); 
    m_armMotorL.setIdleMode(IdleMode.kCoast);
    m_armMotorL.setInverted(true);

    //configures right motor (not inverted) 
    m_armMotorR = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless);
    m_armMotorR.follow(m_armMotorL, true);
    m_armMotorR.setSmartCurrentLimit(30); 
    m_armMotorR.setIdleMode(IdleMode.kCoast);

    m_armEncoder = m_armMotorL.getEncoder(); 

    m_armPidController = new PIDController(0.02, 0, 0);
    m_armEncoder.setPositionConversionFactor(Constants.ArmConstants.ARM_ENCODER_SCALING_FACTOR);
    m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
    m_speakerHeightOffset = 0.0; 
  }

  public void setPower(double power) {
    m_armMotorL.set(power);
  }

  public Command setPowerCommand(double power) {
    return new InstantCommand(() -> {
      setPower(power);
    }, this);
  }

  public Command manualModeCommand(double power) //puts arm into manual mode 
  {
    return new InstantCommand(() -> {
      manualMode(power);
    }, this);
  }

  public void manualMode(double power) 
  {
    if (power > 0 && backLimitSwitch() == false)
    {
      m_angleSetpoint = m_angleSetpoint + 5;
    }
    else if (power < 0)
    {
      m_angleSetpoint = m_angleSetpoint - 5;
    }
  }


  public void setOffset(double increment)
  {
    m_speakerHeightOffset = m_speakerHeightOffset + increment; 
  }

  public Command setOffsetCommand(double increment)
  {
    return new InstantCommand(() -> {
          this.setOffset(increment); 
    }, this);

  }
  public double getOffset()
  {
    return m_speakerHeightOffset; 
  }

  public Command rotateToAngleCommand(double angle) { 
    return runOnce(() -> {
      goToAngle(angle);
    });
  }


  public void goToAngle(double angleDegrees) //sets arm to a specified angle 
  {
    m_angleSetpoint = angleDegrees; 
  }
  
  public void resetPosition() { 
    m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
  }


  public Command resetEncoderCommand() 
  {
    return new InstantCommand(() ->
    {
        resetPosition(); 
    }, this); 
  }

  public Command setEncodertoPropAngle()
  {
    return new InstantCommand(() -> 
    {
      m_armEncoder.setPosition(Constants.ArmConstants.ARM_PROP_ANGLE);
    }, this ); 
  }

  public boolean frontLimitSwitch()
  {
    return !m_frontLimit.get(); 
  }

  public boolean backLimitSwitch()
  {
    return m_backLimit.get(); 
  }

  public double getArmAngle() //returns current angle of the arm 
  {
    return m_armEncoder.getPosition(); 
  }

  @Override
  public void periodic() {
        double rawPower = m_armPidController.calculate(getArmAngle(), m_angleSetpoint);
        double clampedPower = MathUtil.clamp(rawPower, -0.05, 0.1); //clamp power to 8% 
        if (clampedPower > 0)
        {
          if (frontLimitSwitch())
          {
            m_armMotorL.set(0.0);
            m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
          }
          else {
            if (!m_IntakeSubsystem.isIntaking())
            {
              m_armMotorL.set(clampedPower); 
            }
          }
        } else {
          if (backLimitSwitch())
          {
            m_armMotorL.set(0.0); 
          }
          else{
            if (!m_IntakeSubsystem.isIntaking())
            {
              m_armMotorL.set(clampedPower); 
            }
          }
        }

        SmartDashboard.putBoolean("front limit", m_frontLimit.get()); 
        SmartDashboard.putBoolean("bakc ", m_backLimit.get()); 
        m_armMotorL.set(clampedPower);
        //SmartDashboard.putNumber("RawPower", rawPower);
        //SmartDashboard.putNumber("ClampedPower", clampedPower);
    //}
      
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Degrees", getArmAngle()); 
    SmartDashboard.putNumber("ArmSetpoint", m_angleSetpoint);
    SmartDashboard.putNumber("inch Height Offset", m_speakerHeightOffset / 0.0254); 
    //SmartDashboard.putData("ARM PID",m_armPidController);
  }
}
