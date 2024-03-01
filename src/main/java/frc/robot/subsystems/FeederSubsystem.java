// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax m_feederMotor;
  private DigitalInput m_intakeBreaker;
  private DigitalInput m_shooterBreaker;

  // Feeder states are located in the Constants file
  private FeederState m_state;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    m_feederMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
    //m_intakeBreaker = new DigitalInput(FeederConstants.INTAKE_BEAMBREAK_ID); // beam break near intake
    m_shooterBreaker = new DigitalInput(FeederConstants.SHOOTER_BEAMBREAK_ID); // beam break near shooter
    m_state = FeederState.READYINPUT;
    m_feederMotor.set(0.0);
    m_feederMotor.setInverted(true);
  }

  public void setPower(double power)
  {
    m_feederMotor.set(power);
  }

  public Command setFeederPowerCommand(double power) {
    return new InstantCommand(() -> {
      m_feederMotor.set(power);
    }, this);
  }

  public Command switchStateManual(FeederState state)
  {
    return new InstantCommand(() -> {
      m_state = state; 
    }, this);
  }

  // public boolean getIntakeBreaker() {
  //   return m_intakeBreaker.get();
  // }

  public boolean getShooterBreaker() {
    return m_shooterBreaker.get();
  }

  public FeederState getFeederState() {
    return m_state;
  }

  public boolean intakeAllowed() {
    return (m_state == FeederState.READYINPUT) || (m_state == FeederState.INTAKE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putString("FeederState", m_state.name());
    //SmartDashboard.putBoolean("Intake Beam Break", m_intakeBreaker.get());
    SmartDashboard.putBoolean("Shooter Beam Break", m_shooterBreaker.get());

    // if (!m_shooterBreaker.get())
    // {
    //   m_feederMotor.set(0);
    // }

    // switch (m_state) {

    //   case READYINPUT:
    //     // keep the feeder motor off as driver controls intake motor until first beam is
    //     // broken
    //     m_feederMotor.set(0.0);
    //     // if we're in the first beam, change state, making the process autonomous
    //     if (!m_intakeBreaker.get()) {
    //       m_state = FeederState.INTAKE;
    //     }
    //     break;
    //   case INTAKE:
    //     // the note is inside of the breaker beam light
    //     // turn on the feeder to feed it to TRANSIT, but driver should keep intake motor
    //     // on
    //     m_feederMotor.set(FeederConstants.FEEDER_MOTOR_POWER);
    //     // bbbutt, if the note comes out of the first breaker, then begin TRANSIT
    //     if (m_intakeBreaker.get()) {
    //       m_state = FeederState.TRANSIT;
    //     }
    //     break;
    //   case TRANSIT:
    //     // ok now we need to feed the note through the middle section
    //     m_feederMotor.set(FeederConstants.FEEDER_MOTOR_POWER);

    //     // it won't fall, hardware said so :)

    //     // if it hits the second breaker, change to READYSHOOT
    //     if (!m_shooterBreaker.get()) {
    //       m_state = FeederState.READYSHOOT;
    //     }
    //     break;
    //   case READYSHOOT:
    //     // stop feeding, keep note in the top of the feeder, so it is ready to shoot (it
    //     // won't slide)
    //     m_feederMotor.set(0.0);
    //     if (m_shooterBreaker.get()) {
    //       m_state = FeederState.READYINPUT;
    //     }
    //     // shooting is an external action which can be found in the shooter subsystem.
    //     break;
    // }
  }
}
