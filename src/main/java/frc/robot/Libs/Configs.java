package frc.robot.Libs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Configs {

    public static CANSparkMax NEO550(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(20);
        motor.setSecondaryCurrentLimit(20);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static CANSparkMax NEO(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static  DutyCycleEncoder AbsEncbore(DutyCycleEncoder encoder, int port, double conversionFactor) {
        encoder = new DutyCycleEncoder(port);
        encoder.setDistancePerRotation(conversionFactor);
        return encoder;
    }

    private static TalonFX DriveFX(TalonFX motor, double DRIVE_kP, double DRIVE_kS, InvertedValue invert) {
        TalonFXConfigurator configer = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = DRIVE_kP;
        config.Slot0.kS = DRIVE_kS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;        
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configer.apply(config.Slot0);
        configer.apply(config.CurrentLimits);
        configer.apply(config.Voltage);
        configer.apply(config.MotorOutput);
        configer.apply(config.MotionMagic);

        return motor;
    }

    private static TalonFX AzimuthFX (TalonFX motor, double DRIVE_kP, double DRIVE_kD, double DRIVE_kS, InvertedValue invert, MotionMagicConfigs profile) {
        TalonFXConfigurator configer = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = DRIVE_kP;
        config.Slot0.kS = DRIVE_kS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;        
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic = profile;

        configer.apply(config.Slot0);
        configer.apply(config.CurrentLimits);
        configer.apply(config.Voltage);
        configer.apply(config.MotorOutput);
        configer.apply(config.MotionMagic);

        return motor;
      }
}