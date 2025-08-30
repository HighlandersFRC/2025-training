package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(10);
  private final TalonFX rightMotor = new TalonFX(11);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private ElevatorState wantedState = ElevatorState.IDLE;
  private ElevatorState systemState = ElevatorState.IDLE;

  public enum ElevatorState {
    IDLE,
    L1,
    L2,
    L3,
    L4,
    ALGAE_HIGH,
    ALGAE_LOW
  }

  private static final double L1_HEIGHT = 5.0;
  private static final double L2_HEIGHT = 15.0;
  private static final double L3_HEIGHT = 25.0;
  private static final double L4_HEIGHT = 35.0;
  private static final double ALGAE_HIGH_HEIGHT = 20.0;
  private static final double ALGAE_LOW_HEIGHT = 10.0;

  public Elevator() {}

  public void init() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.Slot0.kP = 5.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.MotionMagic.MotionMagicAcceleration = 100;
    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setInverted(true);
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case L1: return ElevatorState.L1;
      case L2: return ElevatorState.L2;
      case L3: return ElevatorState.L3;
      case L4: return ElevatorState.L4;
      case ALGAE_HIGH: return ElevatorState.ALGAE_HIGH;
      case ALGAE_LOW: return ElevatorState.ALGAE_LOW;
      default: return ElevatorState.IDLE;
    }
  }

  private void goToPosition(double rotations) {
    leftMotor.setControl(positionRequest.withPosition(rotations).withVelocity(80).withSlot(0));
    rightMotor.setControl(positionRequest.withPosition(rotations).withVelocity(80).withSlot(0));
  }

  public void setWantedState(ElevatorState state) {
    this.wantedState = state;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    switch (systemState) {
      case L1:
        goToPosition(L1_HEIGHT);
        break;
      case L2:
        goToPosition(L2_HEIGHT);
        break;
      case L3:
        goToPosition(L3_HEIGHT);
        break;
      case L4:
        goToPosition(L4_HEIGHT);
        break;
      case ALGAE_HIGH:
        goToPosition(ALGAE_HIGH_HEIGHT);
        break;
      case ALGAE_LOW:
        goToPosition(ALGAE_LOW_HEIGHT);
        break;
      default:
        leftMotor.setControl(torqueRequest.withOutput(2.0).withMaxAbsDutyCycle(0.1));
        rightMotor.setControl(torqueRequest.withOutput(2.0).withMaxAbsDutyCycle(0.1));
        break;
    }
  }
}
