// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Manipulator.ManipulatorState;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum SuperState {
    DEFAULT,
    AUTO_L1_SCORE,
    AUTO_L2_PLACE,
    AUTO_L2_SCORE,
    AUTO_L3_PLACE,
    AUTO_L3_SCORE,
    AUTO_L4_PLACE,
    AUTO_L4_SCORE,
    HANDOFF,
    MOVE_TO_POINT,
    OUTAKE,
    IDLE
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;
  private boolean pathCompleted = false;

  Drive drive;
  Peripherals peripherals;
  Elevator elevator;
  Straightenator straightenator;
  Arm arm;
  Manipulator manipulator;

  public enum CurrentMode {
    ALGAE,
    CORAL
  }

  public CurrentMode currentMode = CurrentMode.CORAL;

  public Superstructure(Drive driveSubsystem, Peripherals peripheralSubsystem, Elevator elevatorSubsystem,
      Straightenator straightenatorSubsystem, Arm armSubsystem, Manipulator manipulatorSubsystem) {
    drive = driveSubsystem;
    peripherals = peripheralSubsystem;
    elevator = elevatorSubsystem;
    straightenator = straightenatorSubsystem;
    arm = armSubsystem;
    manipulator = manipulatorSubsystem;
  }

  public void setCurrentMode(CurrentMode mode) {
    currentMode = mode;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;

  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean isPathCompleted() {
    return pathCompleted;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        drive.setWantedState(DriveState.DEFAULT);
        handleDefaultState();
        break;
      case OUTAKE:
        manipulator.setWantedState(ManipulatorState.OUTAKE);
        break;
      case AUTO_L1_SCORE:
        handleAutoL1Score();
        break;
      case AUTO_L2_PLACE:
        handleAutoL2Place();
        break;
      case AUTO_L2_SCORE:
        handleAutoL2Score();
        break;
      case AUTO_L3_PLACE:
        handleAutoL3Place();
        break;
      case AUTO_L3_SCORE:
        handleAutoL3Score();
        break;
      case AUTO_L4_SCORE:
        handleAutoL4Score();
        break;
      case AUTO_L4_PLACE:
        handleAutoL4Place();
        break;
      case HANDOFF:
        if (straightenator.isFar()) {
          handleHandOffLowState();
        } else
          handleHandoffState();
        break;
      default:
        handleHandoffState();
        break;
      case IDLE:
        handleIdleState();
        break;
      case MOVE_TO_POINT:
        drive.setWantedState(DriveState.MOVE_TO_POINT);
        break;
    }
  }

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case OUTAKE:
        currentSuperState = SuperState.OUTAKE;
        break;
      case AUTO_L2_PLACE:
        currentSuperState = SuperState.AUTO_L2_PLACE;
        break;
      case AUTO_L2_SCORE:
        currentSuperState = SuperState.AUTO_L2_SCORE;
        break;
      case AUTO_L3_PLACE:
        currentSuperState = SuperState.AUTO_L3_PLACE;
        break;
      case AUTO_L3_SCORE:
        currentSuperState = SuperState.AUTO_L3_SCORE;
        break;
      case AUTO_L4_SCORE:
        currentSuperState = SuperState.AUTO_L4_SCORE;
        break;
      case HANDOFF:
        currentSuperState = SuperState.HANDOFF;
        break;
      case AUTO_L4_PLACE:
        currentSuperState = SuperState.AUTO_L4_PLACE;
        break;
      case AUTO_L1_SCORE:
        currentSuperState = SuperState.AUTO_L1_SCORE;
        break;
      case MOVE_TO_POINT:
        currentSuperState = SuperState.MOVE_TO_POINT;
        break;
    }
    return currentSuperState;
  }

  public void handleHandoffState() {
    drive.setWantedState(DriveState.DEFAULT);
    arm.setWantedState(ArmState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
  }

  public void handleHandOffLowState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_LOW);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);

  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    arm.setWantedState(ArmState.HANDOFF);
  }

  public void handleAutoL1Score() {
    elevator.setWantedState(ElevatorState.AUTO_L1);
    if (elevator.getElevatorPosition() < Constants.inchesToMeters(9.0)) {
      arm.setWantedState(ArmState.HANDOFF);
    } else {
      arm.setWantedState(ArmState.L1_PLACE);
    }
  }

  public void handleAutoL2Place() {
    elevator.setWantedState(ElevatorState.AUTO_L2);
    arm.setWantedState(ArmState.L2_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleAutoL2Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L2);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    arm.setWantedState(ArmState.L2_SCORE);
  }

  public void handleAutoL3Place() {
    elevator.setWantedState(ElevatorState.AUTO_L3);
    arm.setWantedState(ArmState.L3_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleAutoL3Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L3);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    arm.setWantedState(ArmState.L3_SCORE);
  }

  public void handleAutoL4Place() {
    elevator.setWantedState(ElevatorState.AUTO_L4);
    arm.setWantedState(ArmState.L4_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);

  }

  public void handleAutoL4Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L4);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    arm.setWantedState(ArmState.L4_SCORE);

  }

  public void handleIdleState() {

  }

  @Override
  public void periodic() {
    drive.setWantedState(DriveState.DEFAULT);
    Logger.recordOutput("SuperStructure State", currentSuperState);
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}