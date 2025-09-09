// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armMotor = new TalonFX(Constants.CANInfo.ARM_PIVOT_MOTOR_ID);
  private final TalonFX pivotCANcoder = new TalonFX(Constants.CANInfo.PIVOT_CANCODER_ID);
  private double pivotJerk = 0;
  private final double pivotAcceleration = 6.0 * Constants.Ratios.PIVOT_GEAR_RATIO;
  private final double pivotCruiseVelocity = 6.0 * Constants.Ratios.PIVOT_GEAR_RATIO;

  public enum ArmState {
    DEFAULT,
    L4_SCORE,
    L4_PLACE,
    L3_SCORE,
    L3_PLACE,
    L2_SCORE,
    L2_PLACE,
    IDLE
  }

  public Arm() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = 100.0;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 5.0;
    pivotConfig.Slot1.kP = 30.0;
    pivotConfig.Slot1.kI = 0.0;
    pivotConfig.Slot1.kD = 5.0;
    pivotConfig.Slot2.kP = 50.0;
    pivotConfig.Slot2.kI = 0.0;
    pivotConfig.Slot2.kD = 15.0;
    pivotConfig.MotionMagic.MotionMagicJerk = this.pivotJerk;
    pivotConfig.MotionMagic.MotionMagicAcceleration = this.pivotAcceleration;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = this.pivotCruiseVelocity;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
    pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotConfig.Feedback.RotorToSensorRatio = Constants.Ratios.PIVOT_GEAR_RATIO;
  }

  public void init() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
