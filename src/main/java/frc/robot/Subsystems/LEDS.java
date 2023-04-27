package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDS extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int lastR;
  private int lastG;
  private int lastB;

  public LEDS() {
    m_ledBuffer = new AddressableLEDBuffer(240);
    m_led = new AddressableLED(0);
  }

  public Command flash(int r, int g, int b) {
    InstantCommand flash = new InstantCommand(() -> setLEDS(r, g, b), this);
    InstantCommand returnToLast = new InstantCommand(() -> setLEDS(lastR, lastG, lastB), this);

    return new SequentialCommandGroup(flash, new WaitCommand(1), returnToLast);
  }

  public void setLEDS(int r, int g, int b) {
    for(int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);

    lastR = r;
    lastG = g;
    lastB = b;

    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
