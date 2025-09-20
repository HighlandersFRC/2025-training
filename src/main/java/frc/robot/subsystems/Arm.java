package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.math.PID;

public class Arm extends SubsystemBase {
  private final TalonFX armMotor;
  private final PID armPID = new PID(0.04, 0.0, 0.0);

  private static final double ROT_NEG90_DEG = 0;
  private static final double ROT_POS90_DEG = -0.464;
  private static final double CAL_M = (90.0 - (-90.0)) / (ROT_POS90_DEG - ROT_NEG90_DEG);
  private static final double CAL_B = -90.0 - CAL_M * ROT_NEG90_DEG;

  public enum ArmState {
    DEFAULT,
    L4_SCORE,
    L4_PLACE,
    L3_SCORE,
    L3_PLACE,
    L2_SCORE,
    L2_PLACE,
    L1_PLACE,
    HANDOFF,
    IDLE
  }

  private ArmState wantedState = ArmState.DEFAULT;
  private ArmState systemState = ArmState.DEFAULT;

  public Arm() {
    armMotor = new TalonFX(Constants.CANInfo.ARM_PIVOT_MOTOR_ID, new CANBus(Constants.CANInfo.CANBUS_NAME));
    armMotor.setPosition(0);
  }

  public void zeroOnEnable() {
    armMotor.setPosition(0.0);
  }

  public void init() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;

    pivotConfig.Voltage.PeakForwardVoltage = 12.0;
    pivotConfig.Voltage.PeakReverseVoltage = -12.0;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.PIVOT_GEAR_RATIO;

    armMotor.getConfigurator().apply(pivotConfig);
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    armPID.setMaxOutput(10.0);
    armPID.setMinOutput(-10.0);
  }

  public double getArmRotations() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public double getRotorRotations() {
    return armMotor.getRotorPosition().getValueAsDouble();
  }

  public double getArmDegrees() {
    double rotations = getArmRotations();
    return CAL_M * rotations + CAL_B;
  }

  private double degreesToRotations(double degrees) {
    return (degrees - CAL_B) / CAL_M;
  }

  public boolean isReadyForHandoff() {
    return getArmDegrees() < -85.0;
  }

  public void setArmDegrees(double degrees) {
    armPID.setSetPoint(degrees);

    Logger.recordOutput("Arm Target Degrees", degrees);
    Logger.recordOutput("Arm Current Degrees", getArmDegrees());
    Logger.recordOutput("Arm Error Degrees", armPID.getError());
  }

  public void setWantedState(ArmState state) {
    wantedState = state;
  }

  private ArmState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return ArmState.IDLE;
      case L4_SCORE:
        return ArmState.L4_SCORE;
      case L4_PLACE:
        return ArmState.L4_PLACE;
      case L3_SCORE:
        return ArmState.L3_SCORE;
      case L3_PLACE:
        return ArmState.L3_PLACE;
      case L2_SCORE:
        return ArmState.L2_SCORE;
      case L2_PLACE:
        return ArmState.L2_PLACE;
      case L1_PLACE:
        return ArmState.L1_PLACE;
      case DEFAULT:
      case HANDOFF:
        return ArmState.HANDOFF;
      default:
        return wantedState;
    }
  }

  public void setCurrentLimit(int limit) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimit = limit;
    cfg.CurrentLimits.SupplyCurrentLimit = limit;
    armMotor.getConfigurator().apply(cfg);
    Logger.recordOutput("Arm Current Limit", limit);
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    switch (systemState) {
      case IDLE:
        armPID.setSetPoint(getArmDegrees());
        break;
      case L4_SCORE:
        setArmDegrees(14.0);
        break;
      case L4_PLACE:
        setArmDegrees(40.0);
        break;
      case L3_SCORE:
        setArmDegrees(35.0);
        break;
      case L3_PLACE:
        setArmDegrees(50.0);
        break;
      case L2_SCORE:
        setArmDegrees(35.0);
        break;
      case L2_PLACE:
        setArmDegrees(50.0);
        break;
      case L1_PLACE:
        setArmDegrees(-25);
        break;
      case DEFAULT:
        setArmDegrees(-90.0);
        break;
      case HANDOFF:
        setArmDegrees(-90);
        break;
    }

    double pidOutput = armPID.updatePID(getArmDegrees());
    armMotor.set(-pidOutput);

    Logger.recordOutput("Arm Rotations", getArmRotations());
    Logger.recordOutput("Arm RotorRotations", getRotorRotations());
    Logger.recordOutput("Arm Degrees", getArmDegrees());
    Logger.recordOutput("Arm Wanted State", systemState.toString());
    Logger.recordOutput("Arm Supply Current", armMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Arm Stator Current", armMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Arm Motor Voltage", armMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Arm Velocity", armMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Arm PID Output", pidOutput);
    Logger.recordOutput("Arm Ready For Handoff", isReadyForHandoff());
  }
}