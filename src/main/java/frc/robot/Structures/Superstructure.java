package frc.robot.Structures;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.LEDS;
import frc.robot.Subsystems.PinchersofPower;

public class Superstructure extends SubsystemBase {
  private Arm m_arm;
  private PinchersofPower m_pop;
  private LEDS m_leds;
  private BooleanSupplier m_state;
  private double m_TimeOut1;
  private double m_TimeOut2;
  private double m_TimeOut3;

  public Superstructure(Arm arm, PinchersofPower POP, LEDS leds, BooleanSupplier state) {
    m_arm = arm;
    m_pop = POP;
    m_leds = leds;
    m_state = state;
  }

  public void setTimeOuts(double m_TimeOut1, double m_TimeOut2, double m_TimeOut3) {
    this.m_TimeOut1 = m_TimeOut1;
    this.m_TimeOut2 = m_TimeOut2;
    this.m_TimeOut3 = m_TimeOut3;
  }

  public SequentialCommandGroup moveToAngles(double angle1, double angle2, double angle3) {
    SequentialCommandGroup moveBiscep = new SequentialCommandGroup(m_arm.moveBiscep(angle1).withTimeout(m_TimeOut1));
    SequentialCommandGroup moveElbow = new SequentialCommandGroup(m_arm.moveElbow(angle2).withTimeout(m_TimeOut2) );
    SequentialCommandGroup moveWrist = new SequentialCommandGroup(m_arm.moveWrist(angle3).withTimeout(m_TimeOut3));

    return new SequentialCommandGroup(moveBiscep, moveElbow, moveWrist);
  }

  public InstantCommand Intake() {
    return m_pop.intakeCommand(ConeOrCube());
  }

  public InstantCommand Outtake() {
    return m_pop.outtakeCommand(ConeOrCube());
  }

  public InstantCommand Notake() {
    return m_pop.notakeCommand(ConeOrCube());
  }

  public boolean ConeOrCube() {
    return (m_pop.stateCheck() || m_state.getAsBoolean());
  }

  public void ledState() {
    m_leds.setState(ConeOrCube());
  }

  
  @Override
  public void periodic() {}
}
