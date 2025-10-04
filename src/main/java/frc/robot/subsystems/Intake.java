// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX roller = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  private final TalonFX pivot = new TalonFX(Constants.CANInfo.INTAKE_PIVOT_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  private final TorqueCurrentFOC m_torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final PositionTorqueCurrentFOC m_positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  private IntakeState wantedState = IntakeState.DEFAULT;
  private IntakeState systemState = IntakeState.DEFAULT;

  public enum IntakeState {
    INTAKING,
    OUTTAKING,
    DEFAULT,
    DOWN,
    IDLE
  }

  public Intake() {
    init();
  }

  public void init() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.Slot0.kP = 4.068;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.7;
    config.Slot1.kP = 4;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.3;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = 0;
    config.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;
    roller.getConfigurator().apply(config);
    roller.setNeutralMode(NeutralModeValue.Brake);
    pivot.getConfigurator().apply(config);
    pivot.setNeutralMode(NeutralModeValue.Brake);
    pivot.setPosition(0);

  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case INTAKING:
        return IntakeState.INTAKING;
      case OUTTAKING:
        return IntakeState.OUTTAKING;
      case IDLE:
        return IntakeState.IDLE;
      case DEFAULT:
        return IntakeState.DEFAULT;
      case DOWN:
        return IntakeState.DOWN;
      default:
        return IntakeState.IDLE;
    }
  }

  public double getRollerTorque() {
    return roller.getTorqueCurrent().getValueAsDouble();
  }

  public void pivotWithTorque(double current, double maxPercent) {
    pivot.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void pivotToPosition(double pivotRotations) {
    pivot.setControl(m_positionTorqueCurrentFOCRequest
        .withPosition(pivotRotations)
        .withSlot(0));
  }

  public void pivotToPositionHighPower(double pivotRotations) {
    pivot.setControl(m_positionTorqueCurrentFOCRequest
        .withPosition(pivotRotations)
        .withSlot(1));
  }

  public void setRollerCurrent(double amps, double maxPercent) {
    roller.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  public void setRollerPercent(double percent) {
    roller.set(percent);
  }

  public double getPosition() {
    return pivot.getPosition().getValueAsDouble() / Constants.Ratios.INTAKE_PIVOT_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    Logger.recordOutput("Intake Pivot Position", pivot.getPosition().getValueAsDouble());
    switch (systemState) {
      case INTAKING:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(60, 0.75);
        break;
      case OUTTAKING:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(-20, 0.5);
        break;
      case IDLE:
        setRollerPercent(0);
        break;
      case DEFAULT:
        setRollerPercent(0);
        pivotToPositionHighPower(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        break;
      case DOWN:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        break;
      default:
        break;
    }
  }
}
