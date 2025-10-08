// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
  /** Creates a new Intake. */
  private final TalonFX manipulatorMotor = new TalonFX(Constants.CANInfo.MANIPULATOR_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  private boolean algaeMode = false;
  private ArmItem armItem = ArmItem.NONE;

  public void updateAlgaeMode(boolean algaeMode) {
    this.algaeMode = algaeMode;
  }

  public enum ArmItem {
    CORAL,
    ALGAE,
    NONE,
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

  public enum ManipulatorState {
    CORAL_INTAKE,
    ALGAE_INTAKE,
    OUTAKE,
    DEFAULT,
    OFF,
  }

  private ManipulatorState wantedState = ManipulatorState.DEFAULT;
  private ManipulatorState systemState = ManipulatorState.DEFAULT;

  public void setIntakeTorque(double current, double maxPercent) {
    manipulatorMotor.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  private boolean firstTimeCoral = true;
  private double coralTime = Timer.getFPGATimestamp();
  private boolean lastCoralValue = false;
  private double switchTime = Timer.getFPGATimestamp();
  private boolean hasCoralSticky = false;

  public boolean hasCoral() {
    // Logger.recordOutput("Has Coral",
    // (intakeMotor.getVelocity().getValueAsDouble() > -10
    // && intakeMotor.getTorqueCurrent().getValueAsDouble() < -15
    // && intakeMotor.getAcceleration().getValueAsDouble() < -100));
    // Logger.recordOutput("Intake Velocity",
    // manipulatorMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake Torque",
    // manipulatorMotor.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake Acceleration",
    // manipulatorMotor.getAcceleration().getValueAsDouble());
    if (Math.abs(manipulatorMotor.getVelocity().getValueAsDouble()) < 5.0
        && Math.abs(manipulatorMotor.getTorqueCurrent().getValueAsDouble()) > 5.0
    /* && Math.abs(manipulatorMotor.getAcceleration().getValueAsDouble()) < 10 */) {
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
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
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

  public boolean hasCoralForTime(double time) {
    // Logger.recordOutput("Has Coral",
    // (intakeMotor.getVelocity().getValueAsDouble() > -10
    // && intakeMotor.getTorqueCurrent().getValueAsDouble() < -15
    // && intakeMotor.getAcceleration().getValueAsDouble() < -100));
    // Logger.recordOutput("Intake Velocity",
    // manipulatorMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake Torque",
    // manipulatorMotor.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake Acceleration",
    // manipulatorMotor.getAcceleration().getValueAsDouble());
    if (hasCoral() && Timer.getFPGATimestamp() - coralTime > time) {
      return true;
    } else {
      return false;
    }
  }

  public ArmItem getArmItem() {
    // System.out.println(Math.abs(intakeMotor.getAcceleration().getValueAsDouble()));
    if (!algaeMode && manipulatorMotor.getTorqueCurrent().getValueAsDouble() > 5.0) {
      if (Math.abs(manipulatorMotor.getVelocity().getValueAsDouble()) < 5.0) {
        if (Math.abs(manipulatorMotor.getAcceleration().getValueAsDouble()) < 10.0) {
          return ArmItem.CORAL;
        } else {
          return ArmItem.NONE;
        }
      } else {
        return ArmItem.NONE;
      }
    } else if (algaeMode && manipulatorMotor.getTorqueCurrent().getValueAsDouble() > 1.0) {
      if (Math.abs(manipulatorMotor.getVelocity().getValueAsDouble()) < 15.0) {
        if (true) {
          return ArmItem.ALGAE;
        }
      } else {
        return ArmItem.NONE;
      }
    }
    return ArmItem.NONE;
  }

  public Manipulator() {
  }

  public void setIntakePercent(double percent) {
    manipulatorMotor.set(percent);
  }

  public double getIntakeRPS() {
    return manipulatorMotor.getVelocity().getValueAsDouble();
  }

  boolean inL1State = false;
  double initOutakeL1Position = 0.0;
  boolean initOutakeL1 = false;

  private ManipulatorState handleStateTransition() {
    // System.out.println("current: " + intakeMotor.getPosition().getValueAsDouble()
    // / 12.5);
    // System.out.println("init: " + initOutakeL1Position);
    // System.out.println("bool: " + initOutakeL1);
    Logger.recordOutput("povup presses", OI.driverPOVUp.getAsBoolean());
    // System.out.println("in l1 state: " + inL1State);
    if (!OI.driverPOVUp.getAsBoolean() || !OI.driverLT.getAsBoolean()) {
      inL1State = false;
      initOutakeL1Position = 0.0;
      initOutakeL1 = false;
    }
    if (OI.driverLT.getAsBoolean()) {
      if (OI.driverPOVUp.getAsBoolean()) {
        if (!initOutakeL1) {
          initOutakeL1Position = manipulatorMotor.getPosition().getValueAsDouble() / 12.5;
          initOutakeL1 = true;
        }

        if (Math.abs(
            Math.abs(manipulatorMotor.getPosition().getValueAsDouble()) / 12.5
                - Math.abs(initOutakeL1Position)) < 0.5) {
          return ManipulatorState.OUTAKE;
        } else {
          return ManipulatorState.OFF;
        }
      } else {
        initOutakeL1 = false;
        return ManipulatorState.OUTAKE;
      }
    }
    switch (wantedState) {
      case CORAL_INTAKE:
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

  @Override
  public void periodic() {
    Logger.recordOutput("Manipulator Motor Current", manipulatorMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput("Manipulator Torque Current", manipulatorMotor.getStatorCurrent().getValueAsDouble());
    if (armItem != getArmItem()) {
      armItem = getArmItem();
    }
    systemState = handleStateTransition();
    // System.out.println("Intake Current: " +
    // intakeMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Manipulator Has coral", hasCoral());
    Logger.recordOutput("Intake Item", armItem);
    // Logger.recordOutput("Has Coral", hasCoral());
    switch (systemState) {
      case CORAL_INTAKE:
        switch (armItem) {
          case CORAL:
            // if (timeSinceItemSwitch > 1.0) {
            setIntakeTorque(30, 1.0);
            // } else {
            // setIntakePercent(1.0);
            // }
            break;
          default:
            setIntakePercent(1.0);
            break;
        }
        break;
      case ALGAE_INTAKE:
        // System.out.println("algae running");
        switch (armItem) {
          case ALGAE:
            // if (timeSinceItemSwitch > 1.0) {
            setIntakeTorque(55, 0.8);
            // } else {
            // setIntakePercent(1.0);
            // }
            break;
          default:
            setIntakeTorque(60, 0.8);
            break;
        }
        break;
      case OUTAKE:
        switch (armItem) {
          // case CORAL:
          // setIntakePercent(-0.5);
          // break;
          // case ALGAE:
          // setIntakePercent(0.5);
          // break;
          default:
            if (algaeMode) {
              if (OI.driverPOVUp.getAsBoolean()) {
                setIntakePercent(-0.4);
              } else {
                setIntakePercent(-0.35);
              }
            } else {
              if (OI.driverPOVLeft.getAsBoolean()) {
                setIntakePercent(-0.5);
              } else if (OI.driverPOVDown.getAsBoolean()) {
                setIntakePercent(-0.4);
              } else if (OI.driverPOVUp.getAsBoolean()) {
                setIntakePercent(-0.25);
              } else if (OI.driverPOVRight.getAsBoolean()) {
                setIntakePercent(-0.5);
              } else {
                setIntakePercent(-1.0);

              }
            }
            break;
        }
        break;
      case OFF:
        setIntakePercent(0.0);
        break;
      default:
        // System.out.println("Motor Current: " + intakeMotor.getTorqueCurrent());
        if (algaeMode) {
          setIntakeTorque(67, 0.4);
        } else {
          setIntakeTorque(20, 0.2);
        }
    }
  }
}
