package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.tools.controlloops.PID;

public class Pivot extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.PIVOT_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final PID pivotPID = new PID(0.03, 0.0, 0.005);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  private static final double ROT_NEG90_DEG = 0;
  private static final double ROT_POS90_DEG = -0.464;
  private static final double CAL_M = (90.0 - (-90.0)) / (ROT_POS90_DEG - ROT_NEG90_DEG);
  private static final double CAL_B = -90.0 - CAL_M * ROT_NEG90_DEG;

  public enum PivotState {
    DEFAULT,
    ZERO,
    L4_SCORE,
    L4_PLACE,
    L3_SCORE,
    L3_PLACE,
    L2_SCORE,
    L2_PLACE,
    L1_PLACE,
    HORIZONTAL,
    VERTICAL,
    NET,
    HANDOFF,
    IDLE
  }

  private PivotState wantedState = PivotState.DEFAULT;
  private PivotState systemState = PivotState.DEFAULT;

  public Pivot() {
    // pivotMotor.setPosition(0);
  }

  public void zeroOnEnable() {
    pivotMotor.setPosition(0.0);
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

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    pivotPID.setMaxOutput(10.0);
    pivotPID.setMinOutput(-10.0);

    zeroOnEnable();
  }

  public double getPivotPosition() {
    return (pivotMotor.getPosition().getValueAsDouble());
  }

  public double getPivotRotations() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getRotorRotations() {
    return pivotMotor.getRotorPosition().getValueAsDouble();
  }

  public double getPivotDegrees() {
    double rotations = getPivotRotations();
    return CAL_M * rotations + CAL_B;
  }

  private double degreesToRotations(double degrees) {
    return (degrees - CAL_B) / CAL_M;
  }

  public boolean isReadyForHandoff() {
    return getPivotDegrees() < -85.0;
  }

  public void setPivotDegrees(double degrees) {
    pivotPID.setSetPoint(degrees);

    Logger.recordOutput("Pivot Target Degrees", degrees);
    Logger.recordOutput("Pivot Current Degrees", getPivotDegrees());
  }

  public void setWantedState(PivotState state) {
    wantedState = state;
  }

  private PivotState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return PivotState.IDLE;
      case ZERO:
        return PivotState.ZERO;
      case L4_SCORE:
        return PivotState.L4_SCORE;
      case L4_PLACE:
        return PivotState.L4_PLACE;
      case L3_SCORE:
        return PivotState.L3_SCORE;
      case L3_PLACE:
        return PivotState.L3_PLACE;
      case L2_SCORE:
        return PivotState.L2_SCORE;
      case L2_PLACE:
        return PivotState.L2_PLACE;
      case L1_PLACE:
        return PivotState.L1_PLACE;
      case DEFAULT:
        return PivotState.DEFAULT;
      case HORIZONTAL:
        return PivotState.HORIZONTAL;
      case NET:
        return PivotState.NET;
      case HANDOFF:
        return PivotState.HANDOFF;
      case VERTICAL:
        return PivotState.VERTICAL;
      default:
        return wantedState;
    }
  }

  public void setCurrentLimit(int limit) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimit = limit;
    cfg.CurrentLimits.SupplyCurrentLimit = limit;
    pivotMotor.getConfigurator().apply(cfg);
    Logger.recordOutput("Pivot Current Limit", limit);
  }

  public void moveWithTorque(double current, double maxPercent) {
    pivotMotor.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setPivotEncoderPosition(double position) {
    pivotMotor.setPosition(position);
  }

  public boolean getZeroed() {
    if (Math.abs(pivotMotor.getStatorCurrent().getValueAsDouble()) > 10.0
        && Math.abs(pivotMotor.getVelocity().getValueAsDouble()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  private double zeroTime = 0.0;

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    switch (systemState) {
      case IDLE:
        pivotPID.setSetPoint(getPivotDegrees());
        break;
      case ZERO:
        moveWithTorque(-30, 0.2);
        if (getZeroed()) {
          setPivotEncoderPosition(0.0);
        }
        break;
      case L4_SCORE:
        setPivotDegrees(Constants.Pivot.L4_Score);
        break;
      case L4_PLACE:
        setPivotDegrees(Constants.Pivot.L4_Place);
        break;
      case L3_SCORE:
        setPivotDegrees(Constants.Pivot.L3_Score);
        break;
      case L3_PLACE:
        setPivotDegrees(Constants.Pivot.L3_Place);
        break;
      case L2_SCORE:
        setPivotDegrees(Constants.Pivot.L2_Score);
        break;
      case L2_PLACE:
        setPivotDegrees(Constants.Pivot.L2_Place);
        break;
      case L1_PLACE:
        setPivotDegrees(Constants.Pivot.L1_Place);
        break;
      case DEFAULT:
        setPivotDegrees(Constants.Pivot.DEFAULT);
        break;
      case HANDOFF:
        setPivotDegrees(Constants.Pivot.HANDOFF);
        break;
      case NET:
        setPivotDegrees(Constants.Pivot.NET);
        break;
      case HORIZONTAL:
        setPivotDegrees(Constants.Pivot.HORIZONTAL);
        break;
      case VERTICAL:
        setPivotDegrees(Constants.Pivot.VERTICAL);
        break;
    }

    double pidOutput = pivotPID.updatePID(getPivotDegrees());
    pivotMotor.set(-pidOutput);

    Logger.recordOutput("Pivot Rotations", getPivotRotations());
    Logger.recordOutput("Pivot RotorRotations", getRotorRotations());
    Logger.recordOutput("Pivot Degrees", getPivotDegrees());
    Logger.recordOutput("Pivot Wanted State", systemState.toString());
    Logger.recordOutput("Pivot Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Pivot Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Pivot Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Pivot Velocity", pivotMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Pivot PID Output", pidOutput);
    Logger.recordOutput("Pivot Ready For Handoff", isReadyForHandoff());
  }
}