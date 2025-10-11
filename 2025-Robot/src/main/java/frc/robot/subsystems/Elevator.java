// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;
import java.util.PrimitiveIterator.OfDouble;

import javax.lang.model.util.ElementScanner14;

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
  private final double elevatorAcceleration = 3000.0;
  private final double elevatorCruiseVelocity = 3000.0;
  private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;

  boolean isZeroed = false;

  public Elevator() {
    left_elevator = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID, new CANBus(Constants.CANInfo.CANBUS_NAME));
    right_elevator = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
        new CANBus(Constants.CANInfo.CANBUS_NAME));
  }

  public boolean getZeroed() {
    if (Math.abs(left_elevator.getStatorCurrent().getValueAsDouble()) > 10.0
        && Math.abs(left_elevator.getVelocity().getValueAsDouble()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  public enum ElevatorState {
    DEFAULT,
    ZERO,
    AUTO_L1,
    AUTO_L2,
    AUTO_L3,
    AUTO_L4,
    AUTO_SCORE_L3,
    AUTO_SCORE_L4,
    L1,
    L2,
    L3,
    L4,
    FEEDER_INTAKE,
    ALGAE_HIGH,
    ALGAE_LOW,
    NET,
    PROCESSOR,
    GROUND_CORAL_INTAKE,
    GROUND_ALGAE_INTAKE,
    SCORE_L1,
    SCORE_L2,
    AUTO_SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    OVER,
    LOLLIPOP,
    PREHANDOFF,
    HANDOFF_HIGH,
    HANDOFF_LOW
  }

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    double elevatorMultiplier = 1;
    elevatorConfig.Slot0.kP = 0.8 * elevatorMultiplier;
    elevatorConfig.Slot0.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot0.kD = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot0.kG = 1 * elevatorMultiplier;
    elevatorConfig.Slot1.kP = 1.7 * elevatorMultiplier;
    elevatorConfig.Slot1.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot1.kD = 0.4 * elevatorMultiplier;
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

  public void rezeroElevator() {
    isZeroed = false;
  }

  public void moveElevatorToPosition(double position) {
    if (position > Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      left_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position)).withSlot(1));
      right_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position)).withSlot(1));
    } else {
      left_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position)).withSlot(1));
      right_elevator.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position)).withSlot(1));

    }

  }

  public void setWantedState(ElevatorState wantedState) {
    // System.out.println("elevator wanted: " + wantedState);
    // if (isZeroed) {
    this.wantedState = wantedState;
    // } else {
    // this.wantedState = ElevatorState.ZERO;
    // }
  }

  public void moveWithTorque(double current, double maxPercent) {
    left_elevator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    right_elevator.setControl(torqueCurrentFOCRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        // if (isZeroed) {
        return ElevatorState.DEFAULT;
      // } else
      // return ElevatorState.ZERO;
      case ZERO:
        // if (isZeroed) {
        // return ElevatorState.DEFAULT;
        // } else
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
      case AUTO_SCORE_L4:
        return ElevatorState.AUTO_SCORE_L4;
      case FEEDER_INTAKE:
        return ElevatorState.FEEDER_INTAKE;
      case ALGAE_HIGH:
        return ElevatorState.ALGAE_HIGH;
      case ALGAE_LOW:
        return ElevatorState.ALGAE_LOW;
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
      case HANDOFF_HIGH:
        return ElevatorState.HANDOFF_HIGH;
      case HANDOFF_LOW:
        return ElevatorState.HANDOFF_LOW;
      default:
        return ElevatorState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    // System.out.println("Elevator Positon Rotations: " +
    // left_elevator.getPosition().getValueAsDouble());
    // System.out.println("Elevator Positon Meters: "
    // +
    // Constants.Ratios.elevatorRotationsToMeters(left_elevator.getPosition().getValueAsDouble()));
    systemState = handleStateTransition();
    switch (systemState) {
      case DEFAULT:
        moveWithTorque(0, 0);
        break;
      case AUTO_L1:
        moveElevatorToPosition(Constants.Elevator.AUTO_L1);
        break;
      case AUTO_L2:
        moveElevatorToPosition(Constants.Elevator.AUTO_L2);
        break;
      case AUTO_L3:
        moveElevatorToPosition(Constants.Elevator.AUTO_L3);
        break;
      case AUTO_L4:
        moveElevatorToPosition(Constants.Elevator.AUTO_L4);
        break;
      case AUTO_SCORE_L2:
        moveElevatorToPosition(Constants.Elevator.AUTO_SCORE_L2);
        break;
      case AUTO_SCORE_L3:
        moveElevatorToPosition(Constants.Elevator.AUTO_SCORE_L3);
        break;
      case ALGAE_HIGH:
        moveElevatorToPosition(Constants.Elevator.ALGAE_HIGH);
        break;
      case ALGAE_LOW:
        moveElevatorToPosition(Constants.Elevator.ALGAE_LOW);
        break;
      case PROCESSOR:
        moveElevatorToPosition(Constants.Elevator.PROCESSOR);
        break;
      case NET:
        moveElevatorToPosition(Constants.Elevator.NET);
        break;
      case AUTO_SCORE_L4:
        moveElevatorToPosition(Constants.Elevator.AUTO_SCORE_L4);
        break;
      case GROUND_ALGAE_INTAKE:
        moveElevatorToPosition(Constants.Elevator.GROUND_ALGAE_INTAKE);
        break;
      case ZERO:
        moveWithTorque(-30, 0.25);
        if (left_elevator.getTorqueCurrent().getValueAsDouble() < -20.0
            && Math.abs(left_elevator.getVelocity().getValueAsDouble()) < 1) {
          left_elevator.setPosition(0.0);
          right_elevator.setPosition(0.0);
          isZeroed = true;
        }
        break;
      case HANDOFF_HIGH:
        moveElevatorToPosition(Constants.Elevator.HANDOFF_HIGH);
        break;
      case HANDOFF_LOW:
        moveElevatorToPosition(Constants.Elevator.HANDOFF_LOW);
        break;
      default:
        break;
    }
    Logger.recordOutput("Elevator Torque", left_elevator.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator Velocity", left_elevator.getVelocity().getValueAsDouble());
    Logger.recordOutput("Elevator PID OUtput", left_elevator.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput("Elevator PID Error", left_elevator.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Elevator In Meters", Constants.Ratios.elevatorRotationsToMeters(
        left_elevator.getPosition().getValueAsDouble()));
    Logger.recordOutput("Elevator In Inches", Constants.metersToInches(Constants.Ratios.elevatorRotationsToMeters(
        left_elevator.getPosition().getValueAsDouble())));
    Logger.recordOutput("Elevator PID Target", Constants.inchesToMeters(Constants.Ratios.elevatorRotationsToMeters(
        left_elevator.getClosedLoopReference().getValueAsDouble())));
    Logger.recordOutput("Elevator Left Position ", left_elevator.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevator Right Position ", right_elevator.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevators Zeroed?", isZeroed);
    Logger.recordOutput("Elevator State", systemState);
  }

  public double getElevatorPosition() {
    return Constants.Ratios.elevatorRotationsToMeters(left_elevator.getPosition().getValueAsDouble());
  }
}