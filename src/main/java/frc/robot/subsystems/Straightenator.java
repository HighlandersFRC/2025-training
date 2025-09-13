// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Straightenator extends SubsystemBase {
  /** Creates a new Straightenator. */
  private final TalonFX left_straightenator = new TalonFX(Constants.CANInfo.LEFT_STRAIGHTENATOR_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX right_straightenator = new TalonFX(Constants.CANInfo.RIGHT_STRAIGHTENATOR_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private final DigitalInput closeBeamBreak = new DigitalInput(Constants.CANInfo.CLOSE_BEAM_BREAK_SENSOR);
  private final DigitalInput farBeamBreak = new DigitalInput(Constants.CANInfo.FAR_BEAM_BREAK_SENSOR);

  private StraightenatorState wantedState = StraightenatorState.DEFAULT;
  private StraightenatorState systemState = StraightenatorState.DEFAULT;

  private final double voltageThreshold = 30;
  TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public Straightenator() {
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    left_straightenator.getConfigurator().apply(leftConfig);
    right_straightenator.getConfigurator().apply(rightConfig);

  }

  public enum StraightenatorState {
    DEFAULT,
    IDLE
  }

  public boolean isReady() {
    return true;// !entryBeamBreak.get() && !endBeamBreak.get();
  }

  public boolean isClose() {
    return !closeBeamBreak.get();
  }

  public boolean isFar() {
    return !farBeamBreak.get();
  }

  public double getLeftVoltage() {
    return left_straightenator.getMotorVoltage().getValueAsDouble();
  }

  public double getRightVoltage() {
    return right_straightenator.getMotorVoltage().getValueAsDouble();
  }

  public void moveWithTorque(double current, double maxPercent) {
    left_straightenator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    right_straightenator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public StraightenatorState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return StraightenatorState.IDLE;
      case DEFAULT:
      default:
        return StraightenatorState.IDLE;
    }
  }

  public void setWantedState(StraightenatorState state) {
    this.wantedState = state;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    switch (systemState) {
      case IDLE:
        left_straightenator.set(0);
        right_straightenator.set(0);
        break;
      case DEFAULT:
        double diff = left_straightenator.getTorqueCurrent().getValueAsDouble()
            - right_straightenator.getTorqueCurrent().getValueAsDouble();

        if (Math.abs(diff) <= 2.0) {
          moveWithTorque(40, 0.75);
        } else {
          double adjustment = Math.min(Math.abs(diff) * 0.05, 0.25);
          if (diff > 0 && !isClose()) {
            moveWithTorque(40, 0.6 - adjustment);
          } else if (diff < 0 && !isClose()) {
            moveWithTorque(40 - adjustment * 160, 0.6);
          } else {
            moveWithTorque(40, 0.75);
          }
        }

        if (isClose()) {
          moveWithTorque(20, 0.1);
        }
        if (isFar() && isClose()) {
          moveWithTorque(0, 0);
        }
        break;

      default:
        break;
    }
    org.littletonrobotics.junction.Logger.recordOutput("Left Straightenator Voltage",
        left_straightenator.getTorqueCurrent().getValueAsDouble());
    org.littletonrobotics.junction.Logger.recordOutput("Right Straightenator Voltage",
        right_straightenator.getTorqueCurrent().getValueAsDouble());

    org.littletonrobotics.junction.Logger.recordOutput("Close Beam Break", closeBeamBreak.get());
    org.littletonrobotics.junction.Logger.recordOutput("Far Beam Break", farBeamBreak.get());
  }
}