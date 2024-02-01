package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GrabNoteCommand extends Command {
    private enum State {
        hasNotTouchedFirst,
        inFirst,
        outOfFirst,
        inSecond,
        outOfSecond,
        done
    }
    
    private IntakeSubsystem m_intakeSubsystem;
    private FeederSubsystem m_feederSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private State m_State;
    private int m_ShooterTicks = 0;

    private static final int EXECUTE_MS = 20;
    private static final int SHOOT_DURATION_MS = 1000;
    private static final int SHOOTER_TICKS = SHOOT_DURATION_MS / EXECUTE_MS;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabNoteCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_State = m_intakeSubsystem.getFirstBreaker() ? State.inFirst : State.hasNotTouchedFirst;
    m_ShooterTicks = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_State) {
        case hasNotTouchedFirst:
            // if not touched the first beam break, then run the intake at full power
            // so that it sucks the thingy into the beam
            m_intakeSubsystem.setPower(0.04);

            // if we're in the first beam, change state
            if (m_intakeSubsystem.getFirstBreaker()) {
                m_State = State.inFirst;
            }
            break;
        case inFirst:
            // the note is inside of the breaker beam light
            // keep sucking it in with the intake
            m_intakeSubsystem.setPower(0.04);

            // bbbutt, if the note comes out of the first breaker, then change state
            if (!m_intakeSubsystem.getFirstBreaker()) {
                m_State = State.outOfFirst;
            }
            break;
        case outOfFirst:
            // ok now we need to feed the note through the middle section
            m_intakeSubsystem.setPower(0);
            m_feederSubsystem.setPower(0.04);
            
            // if it falls into the intake again :(
            if (m_intakeSubsystem.getFirstBreaker()) {
                m_State = State.inFirst;
                m_feederSubsystem.setPower(0);
            }

            // if it hits the second breaker, move state
            if (m_feederSubsystem.getSecondBreaker()) {
                m_State = State.inSecond;
            }

            break;
        case inSecond:
            // keep feeding it through until it's out of the second breaker
            m_feederSubsystem.setPower(0.04);
            if (!m_feederSubsystem.getSecondBreaker()) {
                m_State = State.outOfSecond;
            }

            break;
        case outOfSecond:
            // feeder continue running

            m_shooterSubsystem.setPower(0.04);
            m_ShooterTicks++;

            // ok im trying to make it last for 1 second but this doesn't work well :(
            if (m_ShooterTicks >= SHOOTER_TICKS) {
                m_State = State.done;
                m_shooterSubsystem.setPower(0);
                m_feederSubsystem.setPower(0);
            }

            break;
        case done:
            break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setPower(0);
    m_feederSubsystem.setPower(0);
    m_shooterSubsystem.setPower(0);
    m_State = State.done;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_State == State.done;
  }
}
