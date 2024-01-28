package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// CODE BASED ON FRC CHARGED UP 2023

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_LED = new AddressableLED(8);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(17);

    public LEDSubsystem() {
        // Green
        setRGB(-1, 0, 255, 0);

        m_LED.setLength(m_LEDBuffer.getLength());
        m_LED.setData(m_LEDBuffer);
        m_LED.start();
    }

    @Override
    public void periodic() {
        m_LED.setData(m_LEDBuffer);
    }

    public void setRGB(int ledIndex, int r, int g, int b) {
        if (ledIndex == -1) {
            int nLeds = getNumberOfLEDs();
            for (int i = 0; i < nLeds; ++i) {
                m_LEDBuffer.setRGB(i, r, g, b);
            }
        }
        else {
            m_LEDBuffer.setRGB(ledIndex, r, g, b);
        }
    }

    public int getNumberOfLEDs() {
        return m_LEDBuffer.getLength();
    }
}