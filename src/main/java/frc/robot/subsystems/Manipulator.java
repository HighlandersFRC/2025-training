// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private final TalonFX manipulatorMotor = new TalonFX(Constants.CANInfo.ARM_MANIPULATOR_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private ManipulatorState wantedState = ManipulatorState.DEFAULT;
  private ManipulatorState systemState = ManipulatorState.DEFAULT;

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0);
  CurrentGamePiece currentGamePiece = CurrentGamePiece.NONE;
  private boolean algaeMode = false;

  public Manipulator() {
    init();
  }

  public void init() {
    TalonFXConfiguration manipulatorConfig = new TalonFXConfiguration();
    manipulatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    manipulatorConfig.CurrentLimits.StatorCurrentLimit = 80;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimit = 80;
    manipulatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    manipulatorMotor.getConfigurator().apply(manipulatorConfig);
    manipulatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public enum CurrentGamePiece {
    CORAL,
    ALGAE,
    NONE
  }

  public enum ManipulatorState {
    CORAL_INTAKE,
    ALGAE_INTAKE,
    OUTAKE,
    DEFAULT,
    OFF,
  }

  private ManipulatorState handleStateTransition() {
    switch (wantedState) {
      case CORAL_INTAKE:
        if (manipulatorMotor.getTorqueCurrent().getValueAsDouble() > 30) {
          currentGamePiece = CurrentGamePiece.CORAL;
          return ManipulatorState.DEFAULT;
        }
        return ManipulatorState.CORAL_INTAKE;
      case ALGAE_INTAKE:
        return ManipulatorState.ALGAE_INTAKE;
      case OUTAKE:
        return ManipulatorState.OUTAKE;
      case OFF:
        return ManipulatorState.OFF;
      default:
        return ManipulatorState.DEFAULT;
    }
  }

  public void setWantedState(ManipulatorState wantedState) {
    this.wantedState = wantedState;
  }

  public void setIntakePercent(double percent) {
    manipulatorMotor.set(percent);
  }

  public double getIntakeRPS() {
    return manipulatorMotor.getVelocity().getValueAsDouble();
  }

  public void setIntakeTorque(double currentAmps, double maxDutyFraction) {
    manipulatorMotor.setControl(torqueCurrentFOCRequest
        .withOutput(currentAmps)
        .withMaxAbsDutyCycle(maxDutyFraction));
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    double motorVelocity = getIntakeRPS();

    switch (systemState) {
      case CORAL_INTAKE:
        setIntakeTorque(40, 0.4);
        break;
      case ALGAE_INTAKE:
        if (Math.abs(motorVelocity) < 25) {
          setIntakeTorque(10, 0.05);
        } else {
          setIntakeTorque(67, 0.3);
        }
        break;
      case OUTAKE:
        setIntakeTorque(-30, 0.3);
        break;
      case OFF:
        setIntakeTorque(0, 0);
        break;
      default:
        setIntakeTorque(20, 0.01);
        break;
    }

    org.littletonrobotics.junction.Logger.recordOutput("Manipulator State", systemState);
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Velocity", motorVelocity);
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Torque",
        manipulatorMotor.getTorqueCurrent().getValueAsDouble());
  }
}