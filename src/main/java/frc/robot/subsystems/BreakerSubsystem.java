package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakerSubsystem extends SubsystemBase {
    private DigitalInput firstBreaker;
    private DigitalInput secondBreaker;

    public BreakerSubsystem() {
        firstBreaker = new DigitalInput(0);
        secondBreaker = new DigitalInput(1);
    }
    
    @Override
    public void periodic() {

    }

    public boolean getFirst() {
        return firstBreaker.get();
    }
}

