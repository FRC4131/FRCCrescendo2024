// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederState;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax m_IntakeMotor;
  private CANSparkMax m_BeltMotor;
  private DigitalInput m_IntakeBreaker;
  private DigitalInput m_FeederBreaker;

  
  // machine states are located in the Constants file

    
  private IntakeSubsystem m_intakeSubsystem;
  private FeederSubsystem m_feederSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private FeederState m_State;
    private int m_ShooterTicks = 0;

    private static final int EXECUTE_MS = 20;
    private static final int SHOOT_DURATION_MS = 1000;
    private static final int SHOOTER_TICKS = SHOOT_DURATION_MS / EXECUTE_MS;


  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
      m_IntakeMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
      m_BeltMotor = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
      m_IntakeBreaker = new DigitalInput(2); // beambreak
      m_FeederBreaker = new DigitalInput(1); //switch 
      m_State = FeederState.READYINPUT;
      m_ShooterTicks = 0;
      m_IntakeMotor.set(0.0);
      m_BeltMotor.set(0.0);
  }

  public Command setIntakePowerCommand(double power) {
    return new InstantCommand(() -> {
      m_IntakeMotor.set(power);
    }, this);
  }

  public Command setFeederPowerCommand(double power) {
    return new InstantCommand(() -> {
      m_BeltMotor.set(power);
    }, this);
  }

  public boolean getIntakeBreaker() {
    return m_IntakeBreaker.get();
  }

  public boolean getFeederBreaker() { 
    return m_FeederBreaker.get();
  }

  public FeederState getFeederState() {
    return m_State;
  }

  public boolean intakeAllowed(){
    return (m_State == FeederState.READYINPUT) || (m_State == FeederState.FEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("FeederState", m_State.name());
    SmartDashboard.putBoolean("FeederBeamBreak", m_FeederBreaker.get());

    switch (m_State) {

      case READYINPUT:
          // keep the feeder motor off as driver controls intake motor until first beam is broken
          m_BeltMotor.set(0.0);
          // if we're in the first beam, change state, making the process autonomous
          if (!m_IntakeBreaker.get()) {
              m_State = FeederState.FEED;
          }
          break;
      case FEED:
          // the note is inside of the breaker beam light
          // turn on the feeder to feed it to TRANSIT, but driver should keep intake motor on
          m_BeltMotor.set(0.04);
          // bbbutt, if the note comes out of the first breaker, then begin TRANSIT
          if (m_IntakeBreaker.get()) {
              m_State = FeederState.TRANSIT;
          }
          break;
      case TRANSIT:
          // ok now we need to feed the note through the middle section
          m_BeltMotor.set(0.04);
          
          // it won't fall, hardware said so :)
          /*if (m_IntakeBreaker.get()) {
              m_State = State.FEED;
              m_feederSubsystem.setPower(0);
          }*/

          // if it hits the second breaker, change to READYSHOOT
          if (!m_FeederBreaker.get()) {
              m_State = FeederState.READYSHOOT;
          }

          break;
      case READYSHOOT:
          // stop feeding, keep note in the top of the feeder, so it is ready to shoot (it won't slide)
          m_BeltMotor.set(0.0);
          if (m_FeederBreaker.get()) {
              m_State = FeederState.READYINPUT;
          }
          // shooting is an external action which can be found in the shooter subsystem.
          break;

      /*case SHOOT:
          // feeder continue running

          m_shooterSubsystem.setPower(0.04);
          m_ShooterTicks++;

          // ok im trying to make it last for 1 second but this doesn't work well :(
          if (m_ShooterTicks >= SHOOTER_TICKS) {
              m_State = State.DONE;
              m_shooterSubsystem.setPower(0);
              m_feederSubsystem.setPower(0);
          }*/
          

          
  }
  }
}
