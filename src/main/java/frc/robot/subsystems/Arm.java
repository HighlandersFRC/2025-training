// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX arm = new TalonFX(Constants.CANInfo.ARM_PIVOT_MOTOR_ID);

  public enum ArmState {
    DEFAULT,
    L4_HOVER,
    L4_PLACE,
    L3_HOVER,
    L3_PLACE,
    L2_HOVER,
    L2_PLACE,
    IDLE
  }

  public Arm() {

  }

  public void init() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
