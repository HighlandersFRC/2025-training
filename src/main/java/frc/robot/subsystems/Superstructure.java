// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveState;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum SuperState {
    DEFAULT,
    PATH_TO_POINT,
    IDLE
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;
  private boolean pathCompleted = false; 

  Drive drive;
  Peripherals peripherals;

  public Superstructure(Drive driveSubsystem, Peripherals peripheralSubsystem) {
    drive = driveSubsystem;
    peripherals = peripheralSubsystem;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;

    if (wantedState == SuperState.PATH_TO_POINT) {
      pathCompleted = false;
    }
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
      case PATH_TO_POINT:
        handlePathToPointState();
        break;
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
      case PATH_TO_POINT:
        if (currentSuperState == SuperState.PATH_TO_POINT && drive.getAtPosition()) {
          pathCompleted = true;
          currentSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.PATH_TO_POINT;
        }
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;
  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
  }

  public void handlePathToPointState() {
    if (!drive.getAtPosition()) {
      drive.setWantedState(DriveState.PATH_TO_POINT);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleIdleState() {

  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}