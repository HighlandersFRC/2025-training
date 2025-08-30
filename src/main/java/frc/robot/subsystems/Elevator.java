// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX left_elevator;
  private TalonFX right_elevator;

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final double elevatorAcceleration = 1482542976.0;
  private final double elevatorCruiseVelocity = 449929104911.0;
  private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;

  public Elevator() {
    left_elevator = new TalonFX(Constants.CANInfo.LEFT_ELEVATOR_MOTOR_ID, new CANBus(Constants.CANInfo.CANBUS_NAME));
    right_elevator = new TalonFX(Constants.CANInfo.RIGHT_ELEVATOR_MOTOR_ID, new CANBus(Constants.CANInfo.CANBUS_NAME));
  }

  public enum ElevatorState {
    DEFAULT,
    ZERO,
    AUTO_L1,
    AUTO_L2,
    AUTO_L3,
    AUTO_L4,
    AUTO_SCORE_L3,
    AUTO_SCORE_MORE_L3,
    L1,
    L2,
    L3,
    L4,
    FEEDER_INTAKE,
    L2_ALGAE,
    L3_ALGAE,
    GROUND_CORAL_INTAKE,
    GROUND_ALGAE_INTAKE,
    PROCESSOR,
    SCORE_L1,
    SCORE_L2,
    AUTO_SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    NET,
    OVER,
    LOLLIPOP,
    PREHANDOFF,
    HANDOFF
  }

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    double elevatorMultiplier = 1;
    elevatorConfig.Slot0.kP = 1.5 * elevatorMultiplier;
    elevatorConfig.Slot0.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot0.kD = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot0.kG = 1 * elevatorMultiplier;
    elevatorConfig.Slot1.kP = 1.5 * elevatorMultiplier;
    elevatorConfig.Slot1.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot1.kD = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot1.kG = 1 * elevatorMultiplier;

    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = this.elevatorAcceleration;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = this.elevatorCruiseVelocity;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    left_elevator.getConfigurator().apply(elevatorConfig);
    right_elevator.getConfigurator().apply(elevatorConfig);
    left_elevator.setNeutralMode(NeutralModeValue.Brake);
    right_elevator.setNeutralMode(NeutralModeValue.Brake);
    left_elevator.setPosition(0.0);
    right_elevator.setPosition(0.0);
  }

  public void moveElevatorToPosition(double position) {
    if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      left_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position)).withSlot(0));
      right_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position)).withSlot(0));
    } else {
      left_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
              .withSlot(1));
      right_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
              .withSlot(1));
    }
  }

  public void setWantedState(ElevatorState wantedState) {
    this.wantedState = wantedState;
  }

  public void moveWithTorque(double current, double maxPercent) {
    left_elevator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    right_elevator.setControl(torqueCurrentFOCRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ElevatorState.DEFAULT;
      case ZERO:
        return ElevatorState.ZERO;
      case OVER:
        return ElevatorState.OVER;
      case L1:
        return ElevatorState.L1;
      case L2:
        return ElevatorState.L2;
      case L3:
        return ElevatorState.L3;
      case L4:
        return ElevatorState.L4;
      case AUTO_L1:
        return ElevatorState.AUTO_L1;
      case AUTO_L2:
        return ElevatorState.AUTO_L2;
      case AUTO_L3:
        return ElevatorState.AUTO_L3;
      case AUTO_L4:
        return ElevatorState.AUTO_L4;
      case AUTO_SCORE_L3:
        return ElevatorState.AUTO_SCORE_L3;
      case AUTO_SCORE_MORE_L3:
        return ElevatorState.AUTO_SCORE_MORE_L3;
      case FEEDER_INTAKE:
        return ElevatorState.FEEDER_INTAKE;
      case L2_ALGAE:
        return ElevatorState.L2_ALGAE;
      case L3_ALGAE:
        return ElevatorState.L3_ALGAE;
      case GROUND_CORAL_INTAKE:
        return ElevatorState.GROUND_CORAL_INTAKE;
      case GROUND_ALGAE_INTAKE:
        return ElevatorState.GROUND_ALGAE_INTAKE;
      case PROCESSOR:
        return ElevatorState.PROCESSOR;
      case SCORE_L1:
        return ElevatorState.SCORE_L1;
      case SCORE_L2:
        return ElevatorState.SCORE_L2;
      case AUTO_SCORE_L2:
        return ElevatorState.AUTO_SCORE_L2;
      case SCORE_L3:
        return ElevatorState.SCORE_L3;
      case SCORE_L4:
        return ElevatorState.SCORE_L4;
      case NET:
        return ElevatorState.NET;
      case LOLLIPOP:
        return ElevatorState.LOLLIPOP;
      case HANDOFF:
        return ElevatorState.HANDOFF;
      case PREHANDOFF:
        return ElevatorState.PREHANDOFF;
      default:
        return ElevatorState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    System.out.println("Elevator Positon Rotations: " + left_elevator.getPosition().getValueAsDouble());
    System.out.println("Elevator Positon Meters: "
        + Constants.Ratios.elevatorRotationsToMeters(left_elevator.getPosition().getValueAsDouble()));
    systemState = handleStateTransition();
    switch (systemState) {
      case DEFAULT:
        moveWithTorque(0, 0);
        break;
      case AUTO_L2:
        moveElevatorToPosition(Constants.inchesToMeters(15));
        break;
      case AUTO_SCORE_L2:
        moveElevatorToPosition(Constants.inchesToMeters(30));
        break;
      default:
        break;
    }
  }
}
