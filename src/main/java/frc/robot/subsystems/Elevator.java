// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.TorqueCurrentFOC;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {
//   private final TalonFX leftMotor = new TalonFX(10);
//   private final TalonFX rightMotor = new TalonFX(11);
//   private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);
//   private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
//   private ElevatorState wantedState = ElevatorState.IDLE;
//   private ElevatorState systemState = ElevatorState.IDLE;

//   public enum ElevatorState {
//     IDLE,
//     L1,
//     L2,
//     L3,
//     L4,
//     ALGAE_HIGH,
//     ALGAE_LOW
//   }

//   private static final double L1_HEIGHT = 5.0;
//   private static final double L2_HEIGHT = 15.0;
//   private static final double L3_HEIGHT = 25.0;
//   private static final double L4_HEIGHT = 35.0;
//   private static final double ALGAE_HIGH_HEIGHT = 20.0;
//   private static final double ALGAE_LOW_HEIGHT = 10.0;

//   public Elevator() {}

//   public void init() {
//     TalonFXConfiguration config = new TalonFXConfiguration();
//     config.CurrentLimits.StatorCurrentLimitEnable = true;
//     config.CurrentLimits.SupplyCurrentLimitEnable = true;
//     config.CurrentLimits.StatorCurrentLimit = 60;
//     config.CurrentLimits.SupplyCurrentLimit = 60;
//     config.Slot0.kP = 5.0;
//     config.Slot0.kI = 0.0;
//     config.Slot0.kD = 0.1;
//     config.MotionMagic.MotionMagicAcceleration = 100;
//     config.MotionMagic.MotionMagicCruiseVelocity = 80;
//     config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
//     leftMotor.getConfigurator().apply(config);
//     rightMotor.getConfigurator().apply(config);
//     leftMotor.setNeutralMode(NeutralModeValue.Brake);
//     rightMotor.setNeutralMode(NeutralModeValue.Brake);
//     rightMotor.setInverted(true);
//     leftMotor.setPosition(0);
//     rightMotor.setPosition(0);
//   }

//   private ElevatorState handleStateTransition() {
//     switch (wantedState) {
//       case L1: return ElevatorState.L1;
//       case L2: return ElevatorState.L2;
//       case L3: return ElevatorState.L3;
//       case L4: return ElevatorState.L4;
//       case ALGAE_HIGH: return ElevatorState.ALGAE_HIGH;
//       case ALGAE_LOW: return ElevatorState.ALGAE_LOW;
//       default: return ElevatorState.IDLE;
//     }
//   }

//   private void goToPosition(double rotations) {
//     leftMotor.setControl(positionRequest.withPosition(rotations).withVelocity(80).withSlot(0));
//     rightMotor.setControl(positionRequest.withPosition(rotations).withVelocity(80).withSlot(0));
//   }

//   public void setWantedState(ElevatorState state) {
//     this.wantedState = state;
//   }

//   @Override
//   public void periodic() {
//     systemState = handleStateTransition();
//     switch (systemState) {
//       case L1:
//         goToPosition(L1_HEIGHT);
//         break;
//       case L2:
//         goToPosition(L2_HEIGHT);
//         break;
//       case L3:
//         goToPosition(L3_HEIGHT);
//         break;
//       case L4:
//         goToPosition(L4_HEIGHT);
//         break;
//       case ALGAE_HIGH:
//         goToPosition(ALGAE_HIGH_HEIGHT);
//         break;
//       case ALGAE_LOW:
//         goToPosition(ALGAE_LOW_HEIGHT);
//         break;
//       default:
//         leftMotor.setControl(torqueRequest.withOutput(2.0).withMaxAbsDutyCycle(0.1));
//         rightMotor.setControl(torqueRequest.withOutput(2.0).withMaxAbsDutyCycle(0.1));
//         break;
//     }
//   }
// }
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
  private final double elevatorAcceleration = 1482542976.0;
  private final double elevatorCruiseVelocity = 449929104911.0;
  private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;

  boolean isZeroed = false;

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
    AUTO_SCORE_L4,
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
        if (isZeroed) {
          return ElevatorState.DEFAULT;
        } else
          return ElevatorState.ZERO;
      case ZERO:
        if (isZeroed) {
          return ElevatorState.DEFAULT;
        } else
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
        moveElevatorToPosition(Constants.inchesToMeters(17));
      case AUTO_L2:
        moveElevatorToPosition(Constants.inchesToMeters(22));
        break;
      case AUTO_L3:
        moveElevatorToPosition(Constants.inchesToMeters(37));
        break;
      case AUTO_L4:
        moveElevatorToPosition(Constants.inchesToMeters(60));
        break;
      case AUTO_SCORE_L2:
        moveElevatorToPosition(Constants.inchesToMeters(20));
        break;
      case AUTO_SCORE_L3:
        moveElevatorToPosition(Constants.inchesToMeters(35));
        break;
      case AUTO_SCORE_L4:
        moveElevatorToPosition(Constants.inchesToMeters(58));
        break;
      case ZERO:
        moveWithTorque(-40, 0.6);
        if (left_elevator.getTorqueCurrent().getValueAsDouble() < -20.0
            && Math.abs(left_elevator.getVelocity().getValueAsDouble()) < 1) {
          left_elevator.setPosition(0.0);
          right_elevator.setPosition(0.0);
          isZeroed = true;
        }
        break;
      default:
        break;
    }
    Logger.recordOutput("Elevator Torque", left_elevator.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator Velocity", left_elevator.getVelocity().getValueAsDouble());
    Logger.recordOutput("Elevators Zeroed?", isZeroed);
    Logger.recordOutput("Elevator State", systemState);
  }
}