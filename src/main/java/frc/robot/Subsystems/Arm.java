package frc.robot.Subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;
import frc.robot.Libs.DoubleJointedArmFeedforward;

public class Arm extends SubsystemBase {
  private CANSparkMax m_Biscep;
  private CANSparkMax m_Elbow;
  private CANSparkMax m_Wrist;
  private DutyCycleEncoder m_BiscepEncoder;
  private DutyCycleEncoder m_ElbowEncoder;
  private DutyCycleEncoder m_WristEncoder;
  private TrapezoidProfile.Constraints m_BiscepConstraints;
  private TrapezoidProfile.Constraints m_ElbowConstraints;
  private TrapezoidProfile.Constraints m_WristConstraints;
  private ProfiledPIDController m_BiscepController;
  private ProfiledPIDController m_ElbowController;
  private ProfiledPIDController m_WristController;
  private DoubleJointedArmFeedforward.JointConfig m_BiscepConfig;
  private DoubleJointedArmFeedforward.JointConfig m_ElbowConfig;
  private DoubleJointedArmFeedforward m_DoubleJointFF;
  private ArmFeedforward m_WristFF;
  private double m_BiscepVelRad;
  private double m_ElbowVelRad;
  private double m_WristVelRad;

  public Arm() {
    m_Biscep = Configs.NEO(m_Biscep, 0,  false);
    m_Elbow = Configs.NEO(m_Elbow, 1, false);
    m_Wrist = Configs.NEO(m_Wrist, 2, false);

    m_BiscepEncoder = Configs.AbsEncbore(m_BiscepEncoder, 0, 2*Math.PI);
    m_ElbowEncoder = Configs.AbsEncbore(m_ElbowEncoder, 1, 2*Math.PI);
    m_WristEncoder = Configs.AbsEncbore(m_WristEncoder, 2, 2*Math.PI);

    m_BiscepConstraints = new TrapezoidProfile.Constraints(1.0, 1.0);
    m_ElbowConstraints = new TrapezoidProfile.Constraints(1.0, 1.0);
    m_WristConstraints = new TrapezoidProfile.Constraints(1.0, 1.0);

    m_BiscepController = new ProfiledPIDController(1.5, 0.0, 0.0, m_BiscepConstraints);
    m_ElbowController = new ProfiledPIDController(1.5, 0.0, 0.0, m_ElbowConstraints);
    m_WristController = new ProfiledPIDController(1.5, 0.0, 0.0, m_WristConstraints);

    configPPID(m_BiscepController);
    configPPID(m_ElbowController);
    configPPID(m_WristController);

    m_BiscepConfig = new DoubleJointedArmFeedforward.JointConfig(0.0, 0.0, 0.0, 0.0, DCMotor.getNEO(1));
    m_ElbowConfig = new DoubleJointedArmFeedforward.JointConfig(0.0, 0.0, 0.0, 0.0, DCMotor.getNEO(1));

    m_DoubleJointFF = new DoubleJointedArmFeedforward(m_BiscepConfig, m_ElbowConfig);

    m_WristFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command moveBiscep(double setpointRad) {
    return new FunctionalCommand(
    ()-> resetBiscepProfile(), 
    ()-> setBiscepVolts( biscepCalculate(setpointRad) ), 
    (interrupted)-> setBiscepVolts(0), 
    ()-> m_BiscepController.atGoal(),
    this);
  }

  public Command moveElbow(double setpointRad) {
    return new FunctionalCommand(
    ()-> resetElbowProfile(), 
    ()-> setElbowVolts( elbowCalculate(setpointRad) ), 
    (interrupted)-> setElbowVolts(0), 
    ()-> m_ElbowController.atGoal(),
    this);
  }

  public Command moveWrist(double setpointRad) {
    return new FunctionalCommand(
    ()-> resetWristProfile(), 
    ()-> setWristVolts( wristCalculate(setpointRad) ), 
    (interrupted)-> setWristVolts(0), 
    ()-> m_WristController.atGoal(),
    this);
  }

  public void setBiscepVolts(double volts) {
    m_Biscep.setVoltage(volts);
  }

  public void setElbowVolts(double volts) {
    m_Elbow.setVoltage(volts);
  }

  public void setWristVolts(double volts) {
    m_Wrist.setVoltage(volts);
  }

  public void resetBiscepProfile() {
    m_BiscepController.reset(getBiscepRadians());
  }

  public void resetElbowProfile() {
    m_ElbowController.reset(getElbowRadians());
  }

  public void resetWristProfile() {
    m_WristController.reset(getWristRadians());
  }

  public double biscepCalculate(double setpointRad) {
    double PPID = m_BiscepController.calculate(getBiscepRadians(), setpointRad);
    
    double FF = getDoubleJointFFVector().get(0, 0);

    return MathUtil.clamp(PPID + FF, -12, 12);
  }

  public double elbowCalculate(double setpoint) {
    double PPID = m_ElbowController.calculate(getBiscepRadians(), setpoint);
    
    double FF = getDoubleJointFFVector().get(1, 0);
      
    return MathUtil.clamp(PPID + FF, -12, 12);
  }

  public double wristCalculate(double setpoint) {
    double PPID = m_WristController.calculate(getWristRadians(), setpoint);
    double FF = m_WristFF.calculate(getWristRadians(), getWristRadiansPerSecond());
    return MathUtil.clamp(PPID + FF, -12, 12);
  }

  public double getBiscepRadians() {
    return m_BiscepEncoder.getAbsolutePosition();
  }

  public double getElbowRadians() {
    return m_BiscepEncoder.getAbsolutePosition();
  }

  public double getWristRadians() {
    return m_BiscepEncoder.getAbsolutePosition();
  }

  public double getBiscepRadiansPerSecond() {
    double velocity = (m_Biscep.getEncoder().getVelocity()*2*Math.PI)/60;
    m_BiscepVelRad = velocity;
    return velocity;
  }

  public double getElbowRadiansPerSecond() {
    return (m_Elbow.getEncoder().getVelocity()*2*Math.PI)/60;
  }

  public double getWristRadiansPerSecond() {
    return (m_Wrist.getEncoder().getVelocity()*2*Math.PI)/60;
  }

  public double getBiscepAccRadianPerSecond() {
    double last_V = m_BiscepVelRad;
    double accel = (getBiscepRadiansPerSecond() - last_V)/0.02;
    return accel;
  }

  public double getElbowAccRadianPerSecond() {
    double last_V = m_ElbowVelRad;
    double accel = (getElbowRadiansPerSecond() - last_V)/0.02;
    return accel;
  }

  public double getWristAccRadianPerSecond() {
    double last_V = m_WristVelRad;
    double accel = (getWristRadiansPerSecond() - last_V)/0.02;
    return accel;
  }

  public void resetBiscep() {
    m_BiscepEncoder.reset();
  }

  public void resetElbow() {
    m_ElbowEncoder.reset();
  }

  public void resetWrist() {
    m_WristEncoder.reset();
  }

  public void configPPID(ProfiledPIDController controller) {
    controller.enableContinuousInput(0, 2*Math.PI);
    controller.setTolerance(0.0175);
  }

  public Vector<N2> getDoubleJointFFVector() {
    return m_DoubleJointFF.calculate(
      VecBuilder.fill(getBiscepRadians(), getElbowRadians()), 
      VecBuilder.fill(getBiscepRadiansPerSecond(), getElbowRadiansPerSecond()), 
      VecBuilder.fill(getBiscepAccRadianPerSecond(), getElbowAccRadianPerSecond()));
  }
}