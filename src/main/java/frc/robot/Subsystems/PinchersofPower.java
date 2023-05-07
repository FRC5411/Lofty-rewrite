package frc.robot.Subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;

public class PinchersofPower extends SubsystemBase {
  private CANSparkMax m_Spintake;
  private CANSparkMax m_SpintakeFollower;
  private DoubleSolenoid m_Clamp;
  private Compressor m_Compressor;
  private DigitalInput limitSwitch;

  public PinchersofPower() {
    m_Spintake = Configs.NEO550(m_Spintake, 0,  false);
    m_SpintakeFollower = Configs.NEO550(m_SpintakeFollower, 1, false);

    m_Clamp = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 0, 0);
    m_Compressor = new Compressor(0, PneumaticsModuleType.REVPH);

    limitSwitch = new DigitalInput(0);

    m_SpintakeFollower.follow(m_Spintake);
  }

  public void setSpeed(double speed) {
    m_Spintake.set(speed);
  }

  public void inSpin() {
    m_Spintake.set(0.5);
  }

  public void outSpin() {
    m_Spintake.set(-0.5);
  }

  public void noSpin() {
    m_Spintake.set(0);
  }
  
  public void clamp() {
    if(m_Clamp.get() == DoubleSolenoid.Value.kReverse) {
        m_Clamp.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void unclamp() {
    if(m_Clamp.get() == DoubleSolenoid.Value.kReverse) {
        m_Clamp.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void intake(boolean state) {
    if(state == true) {
      unclamp();
      clamp();
    }
    inSpin();
  }

  public void noTake(boolean state) {
    if(state == true) {
      noSpin();
    }
    inSpin();
  }

  public void outtake(boolean state) {
    if(state == true) {
      unclamp();
      clamp();
    }
    else {
    outSpin();
    }
  }

  public void defaultTake() {
    unclamp();
  }

  public void stop() {
    m_Compressor.disable();
  }

  public void start() {
    m_Compressor.enableDigital();
  }

  public boolean stateCheck() {
    if(limitSwitch.get() == true) {
      return true;
    }
    return false;
  }

  public InstantCommand intakeCommand(boolean state) {
    return new InstantCommand(() -> intake(state));
  }

  public InstantCommand outtakeCommand(boolean state) {
    return new InstantCommand(() -> outtake(state));
  }

  public InstantCommand notakeCommand(boolean state) {
    return new InstantCommand(() -> noTake(state));
  }

  @Override
  public void periodic() {

  }
}
