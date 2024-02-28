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

// arm subsystem controls the pivot of the arm/indexing mechanism. uses 2 motors. in this iteration: left = leader, right = follower

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_armMotorL;  //lead/left motor
  private CANSparkMax m_armMotorR; //follower/right motor
  private RelativeEncoder m_armEncoder; 
  private PIDController m_armPidController; 
  //private ArmFeedforward m_armFeedforward; 
  private double m_angleSetpoint = Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE;
  private boolean m_isManualMode; 

  public ArmSubsystem() {

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
    //m_armFeedforward = new ArmFeedforward(0, 0, 0, 0); 

    m_armEncoder.setPositionConversionFactor(Constants.ArmConstants.ARM_ENCODER_SCALING_FACTOR);
    m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);

    m_isManualMode = false; 
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
      m_isManualMode = true; 
      m_armMotorL.set(power);
    }, this);
  }

  public Command manualModeOffCommand() //takes arm out of manual mode and sets pid to resting angle 
  {
    return new InstantCommand(() -> {
      m_armPidController.setSetpoint(this.getArmAngle());
      m_isManualMode = false; 
      //this.resetPosition();

    }); 
  }

  public Command armJoyStickCommand(DoubleSupplier rotSupplier)
  {
    return new InstantCommand(() -> {
      m_isManualMode = true; 
      if (rotSupplier.getAsDouble() > -0.08 && rotSupplier.getAsDouble() < 0.08)
      {
        m_armMotorL.set(rotSupplier.getAsDouble());
      }
      else if (rotSupplier.getAsDouble() < -0.08)
      {
        m_armMotorL.set(-0.08);
      }
      else if (rotSupplier.getAsDouble() > 0.08)
      {
        m_armMotorL.set(0.08);
      }
    });
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

  public Command resetArmEncoderCommand()
  {
    return new InstantCommand(() ->
    {
      m_armEncoder.setPosition(0.0);
    }); 
  }

   public Command resetArmPositionCommand()
  {
    return new InstantCommand(() ->
    {
      resetPosition(); 
    }, this); 
  }
  
  
  public void resetPosition() { 
    this.goToAngle(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE); 
    m_armEncoder.setPosition(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
  }

  public double getArmAngle() //returns current angle of the arm 
  {
    return m_armEncoder.getPosition(); 
  }

  @Override
  public void periodic() {
    if (!m_isManualMode)
    {
        double rawPower = m_armPidController.calculate(getArmAngle(), m_angleSetpoint);
        double clampedPower = MathUtil.clamp(rawPower, -0.08, 0.08); //clamp power to 8% 
        m_armMotorL.set(clampedPower);
        SmartDashboard.putNumber("RawPower", rawPower);
        SmartDashboard.putNumber("ClampedPower", clampedPower);
    }
      
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Degrees", getArmAngle()); 
    SmartDashboard.putNumber("ArmSetpoint", m_angleSetpoint);
    SmartDashboard.putData("ARM PID",m_armPidController);
  }
}
