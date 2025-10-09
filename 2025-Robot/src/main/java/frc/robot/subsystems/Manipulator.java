// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import javax.lang.model.util.ElementScanner14;

import org.ejml.dense.row.decompose.hessenberg.HessenbergSimilarDecomposition_ZDRM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private final TalonFX manipulatorMotor = new TalonFX(Constants.CANInfo.MANIPULATOR_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private ManipulatorState wantedState = ManipulatorState.DEFAULT;
  private ManipulatorState systemState = ManipulatorState.DEFAULT;

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0);

  private boolean algaeMode = false;
  private ArmItem armItem = ArmItem.NONE;

  private boolean firstTimeCoral = true;
  private double coralTime = Timer.getFPGATimestamp();
  private boolean lastCoralValue = false;
  private double switchTime = Timer.getFPGATimestamp();
  private boolean hasCoralSticky = false;

  public Manipulator() {
    init();
  }

  public void init() {
    TalonFXConfiguration manipulatorConfig = new TalonFXConfiguration();
    manipulatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    manipulatorConfig.CurrentLimits.StatorCurrentLimit = 80;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimit = 80;
    manipulatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    manipulatorMotor.getConfigurator().apply(manipulatorConfig);
    manipulatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public enum ArmItem {
    CORAL,
    ALGAE,
    NONE,
  }

  public enum ManipulatorState {
    CORAL_INTAKE,
    ALGAE_INTAKE,
    OUTAKE,
    DEFAULT,
    OFF,
  }

  private ManipulatorState handleStateTransition() {
    if (OI.driverLT.getAsBoolean()) {
      return ManipulatorState.OUTAKE;
    } else
      switch (wantedState) {
        case CORAL_INTAKE:
          return ManipulatorState.CORAL_INTAKE;
        case ALGAE_INTAKE:
          return ManipulatorState.ALGAE_INTAKE;
        case OUTAKE:
          return ManipulatorState.OUTAKE;
        case OFF:
          return ManipulatorState.OFF;
        case DEFAULT:
          return ManipulatorState.DEFAULT;
        default:
          if (OI.driverLT.getAsBoolean()) {
            return ManipulatorState.OUTAKE;
          } else
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

  public boolean hasCoral() {
    if (Math.abs(manipulatorMotor.getVelocity().getValueAsDouble()) < 4.0
        && Math.abs(manipulatorMotor.getTorqueCurrent().getValueAsDouble()) > 2.0) {
      if (firstTimeCoral) {
        firstTimeCoral = false;
        coralTime = Timer.getFPGATimestamp();
      }
      if (lastCoralValue != true) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Intake Item: Has Coral");
      }
      lastCoralValue = true;
      return true;
    } else {
      firstTimeCoral = true;
      coralTime = Timer.getFPGATimestamp();
      if (lastCoralValue != false) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Intake Item: Empty");
      }
      lastCoralValue = false;
      return false;
    }
  }

  public boolean hasCoralSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.2) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.3) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

  public boolean hasCoralSemiSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

  @Override
  public void periodic() {
    // System.out.println(hasCoral());
    systemState = handleStateTransition();

    double motorVelocity = getIntakeRPS();
    if (OI.driverLT.getAsBoolean()) {
      setIntakeTorque(-30, 0.3);

    } else {
      switch (systemState) {
        case CORAL_INTAKE:
          setIntakeTorque(15, 0.3);
          break;
        case ALGAE_INTAKE:
          setIntakeTorque(67.41, 1.0);
          break;
        case OUTAKE:
          setIntakeTorque(-60, 1.0);
          break;
        case OFF:
          setIntakeTorque(0, 0);
          break;
        default:
          setIntakeTorque(15, 0.1);
          break;
      }
    }

    org.littletonrobotics.junction.Logger.recordOutput("Manipulator State",
        systemState);
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Acceleration",
        manipulatorMotor.getAcceleration().getValueAsDouble());
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Velocity",
        motorVelocity);
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Torque",
        manipulatorMotor.getTorqueCurrent().getValueAsDouble());
    org.littletonrobotics.junction.Logger.recordOutput("Manipulator Has Coral",
        hasCoral());
  }
}