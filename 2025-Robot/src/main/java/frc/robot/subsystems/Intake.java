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
    ZERO,
    IDLE
  }

  public boolean getZeroed() {
    if (Math.abs(pivot.getStatorCurrent().getValueAsDouble()) > 10.0
        && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  public Intake() {
  }

  public void init() {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 80;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 80;
    rollerConfig.Slot0.kP = 4.068;
    rollerConfig.Slot0.kI = 0.0;
    rollerConfig.Slot0.kD = 0.7;
    rollerConfig.Slot1.kP = 4;
    rollerConfig.Slot1.kI = 0.0;
    rollerConfig.Slot1.kD = 0.3;
    rollerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    rollerConfig.Slot0.kG = 0;
    rollerConfig.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
    rollerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfig.Slot0.kP = 4.068;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.7;
    pivotConfig.Slot1.kP = 4;
    pivotConfig.Slot1.kI = 0.0;
    pivotConfig.Slot1.kD = 0.3;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Slot0.kG = 0;
    pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;
    roller.getConfigurator().apply(rollerConfig);
    roller.setNeutralMode(NeutralModeValue.Brake);
    pivot.getConfigurator().apply(pivotConfig);
    pivot.setNeutralMode(NeutralModeValue.Brake);
    pivot.setPosition(0);

  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case INTAKING:
        return IntakeState.INTAKING;
      case ZERO:
        return IntakeState.ZERO;
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
        setRollerCurrent(80, 1);
        break;
      case OUTTAKING:
        // pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(-80, 0.5);
        break;
      case IDLE:
        setRollerPercent(0);
        break;
      case DEFAULT:
        setRollerPercent(0);
        pivotToPositionHighPower(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        break;
      case ZERO:
        setRollerPercent(0);
        pivotWithTorque(40, 0.1);
        if (getZeroed()) {
          pivot.setPosition(0.0);
        }
        break;
      case DOWN:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerPercent(0);
        break;
      default:
        break;
    }
    Logger.recordOutput("Intake State", systemState);
  }
}