// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum SuperState {
    DEFAULT,
    AUTO_L2_PLACE,
    AUTO_L2_SCORE,
    AUTO_L3_PLACE,
    AUTO_L3_SCORE,
    AUTO_L4_PLACE,
    AUTO_L4_SCORE,
    IDLE
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;
  private boolean pathCompleted = false;

  Drive drive;
  Peripherals peripherals;
  Elevator elevator;

  public Superstructure(Drive driveSubsystem, Peripherals peripheralSubsystem, Elevator elevatorSubsystem) {
    drive = driveSubsystem;
    peripherals = peripheralSubsystem;
    elevator = elevatorSubsystem;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;

  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public void resetPathToPoint() {
    pathCompleted = false;
    if (currentSuperState == SuperState.DEFAULT && wantedSuperState == SuperState.DEFAULT) {
      wantedSuperState = SuperState.IDLE;
      currentSuperState = SuperState.IDLE;
    }
  }

  public boolean isPathCompleted() {
    return pathCompleted;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case AUTO_L2_PLACE:
        handleAutoL2Place();
        break;
      case AUTO_L2_SCORE:
        handleAutoL2Score();
      default:
        handleIdleState();
        break;
    }
  }

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case AUTO_L2_PLACE:
        currentSuperState = SuperState.AUTO_L2_PLACE;
        break;
      case AUTO_L2_SCORE:
        currentSuperState = SuperState.AUTO_L2_SCORE;
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;
  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.DEFAULT);
  }

  public void handleAutoL2Place() {
    elevator.setWantedState(ElevatorState.AUTO_L2);
  }

  public void handleAutoL2Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L2);
  }

  public void handleAutoL3Place() {

  }

  public void handleAutoL3Score() {

  }

  public void handleAutoL4Place() {

  }

  public void handleAutoL4Score() {

  }

  public void handleIdleState() {

  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}