// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.IntFunction;

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
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator.ArmItem;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Straightenator.StraightenatorState;

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
    L1_SCORE,
    L2_SCORE,
    L2_PLACE,
    L3_SCORE,
    L3_PLACE,
    L4_SCORE,
    L4_PLACE,
    HANDOFF,
    ALGAE_HIGH,
    ALGAE_LOW,
    ALGAE_HOME,
    NET,
    PROCESSOR,
    MOVE_TO_POINT,
    MANIPULATOR_OUTTAKE,
    OUTTAKE,
    INTAKING,
    IDLE
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;
  private boolean pathCompleted = false;

  public boolean algaeMode = false;

  private boolean manipulatorHasCoral = false;
  private BooleanSupplier coralInManipulator = () -> manipulatorHasCoral;

  Drive drive;
  Peripherals peripherals;
  Elevator elevator;
  Straightenator straightenator;
  Arm arm;
  Manipulator manipulator;
  Intake intake;

  public enum CurrentMode {
    ALGAE,
    CORAL
  }

  public CurrentMode currentMode = CurrentMode.CORAL;

  public Superstructure(Drive driveSubsystem, Peripherals peripheralSubsystem, Elevator elevatorSubsystem,
      Straightenator straightenatorSubsystem, Arm armSubsystem, Manipulator manipulatorSubsystem,
      Intake intakeSubsystem) {
    drive = driveSubsystem;
    peripherals = peripheralSubsystem;
    elevator = elevatorSubsystem;
    straightenator = straightenatorSubsystem;
    arm = armSubsystem;
    manipulator = manipulatorSubsystem;
    intake = intakeSubsystem;
  }

  public BooleanSupplier hasCoralInManipulator() {
    return coralInManipulator;
  }

  public void setManipulatorHasCoral(boolean hasCoral) {
    coralInManipulator = () -> hasCoral;
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
      case MANIPULATOR_OUTTAKE:
        manipulator.setWantedState(ManipulatorState.OUTAKE);
        break;
      case OUTTAKE:
        handleOuttakeState();
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
          if (Constants.metersToInches(elevator.getElevatorPosition()) < Constants.Elevator.HANDOFF_LOW + 1.0) {
            handleHandoffState();
          }
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
      case ALGAE_HIGH:
        handleAlgaeHigh();
        break;
      case ALGAE_LOW:
        handleAlgaeLow();
        break;
      case NET:
        handleNet();
        break;
      case PROCESSOR:
        handleProcessor();
        break;
      case L1_SCORE:
        handleL1Score();
        break;
      case L2_PLACE:
        handleL2Place();
        break;
      case L2_SCORE:
        handleL2Score();
        break;
      case L3_PLACE:
        handleL3Place();
        break;
      case L3_SCORE:
        handleL3Score();
        break;
      case L4_PLACE:
        handleL4Place();
        break;
      case L4_SCORE:
        handleL4Score();
        break;
      case INTAKING:
        handleIntakingState();
        break;
      case ALGAE_HOME:
        handleAlgaeHome();
        break;
    }
  }

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case MANIPULATOR_OUTTAKE:
        currentSuperState = SuperState.MANIPULATOR_OUTTAKE;
        break;
      case OUTTAKE:
        currentSuperState = SuperState.OUTTAKE;
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
      case ALGAE_HIGH:
        currentSuperState = SuperState.ALGAE_HIGH;
        break;
      case ALGAE_LOW:
        currentSuperState = SuperState.ALGAE_LOW;
        break;
      case NET:
        currentSuperState = SuperState.NET;
        break;
      case IDLE:
        currentSuperState = SuperState.IDLE;
        break;
      case INTAKING:
        if (straightenator.isFar()) {
          currentSuperState = SuperState.HANDOFF;
        } else
          currentSuperState = SuperState.INTAKING;
        break;
      case L1_SCORE:
        currentSuperState = SuperState.L1_SCORE;
        break;
      case L2_PLACE:
        currentSuperState = SuperState.L2_PLACE;
        break;
      case L2_SCORE:
        currentSuperState = SuperState.L2_SCORE;
        break;
      case L3_PLACE:
        currentSuperState = SuperState.L3_PLACE;
        break;
      case L3_SCORE:
        currentSuperState = SuperState.L3_SCORE;
        break;
      case L4_PLACE:
        currentSuperState = SuperState.L4_PLACE;
        break;
      case L4_SCORE:
        currentSuperState = SuperState.L4_SCORE;
        break;
      case PROCESSOR:
        currentSuperState = SuperState.PROCESSOR;
        break;
      case ALGAE_HOME:
        currentSuperState = SuperState.ALGAE_HOME;
        break;
      default:
        currentSuperState = SuperState.DEFAULT;
        break;
    }
    return currentSuperState;
  }

  public void handleDefaultState() {
    if (algaeMode) {
      handleAlgaeHome();
    } else {
      handleHandoffState();
    }
  }

  public void handleAlgaeHome() {
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    arm.setWantedState(ArmState.VERTICAL);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
  }

  public void handleHandoffState() {
    drive.setWantedState(DriveState.DEFAULT);
    arm.setWantedState(ArmState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    intake.setWantedState(IntakeState.IDLE);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
  }

  public void handleHandOffLowState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_LOW);
    arm.setWantedState(ArmState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    intake.setWantedState(IntakeState.IDLE);

  }

  // public void handleDefaultState() {
  // drive.setWantedState(DriveState.DEFAULT);
  // elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
  // arm.setWantedState(ArmState.HANDOFF);
  // intake.setWantedState(Intake.IntakeState.DEFAULT);
  // manipulator.setWantedState(ManipulatorState.OFF);
  // }

  public void handleAutoL1Score() {
    elevator.setWantedState(ElevatorState.AUTO_L1);
    if (elevator.getElevatorPosition() < Constants.inchesToMeters(9.0)) {
      arm.setWantedState(ArmState.HANDOFF);
    } else {
      arm.setWantedState(ArmState.L1_PLACE);
    }
  }

  public void handleAutoL2Place() {
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
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
    arm.setWantedState(ArmState.HANDOFF);

    if (arm.isReadyForHandoff()) {
      setWantedState(SuperState.HANDOFF);
    }
  }

  public void handleL1Score() {
    elevator.setWantedState(ElevatorState.AUTO_L1);
    if (elevator.getElevatorPosition() < Constants.inchesToMeters(9.0)) {
      arm.setWantedState(ArmState.HANDOFF);
    } else {
      arm.setWantedState(ArmState.L1_PLACE);
    }
  }

  public void handleL2Place() {
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    arm.setWantedState(ArmState.L2_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleL2Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L2);
    arm.setWantedState(ArmState.L2_SCORE);
  }

  public void handleL3Place() {
    elevator.setWantedState(ElevatorState.AUTO_L3);
    arm.setWantedState(ArmState.L3_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleL3Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L3);
    arm.setWantedState(ArmState.L3_SCORE);
  }

  public void handleL4Place() {
    elevator.setWantedState(ElevatorState.AUTO_L4);
    arm.setWantedState(ArmState.L4_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);

  }

  public void handleL4Score() {
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L4);
    arm.setWantedState(ArmState.HORIZONTAL);
  }

  public void handleAlgaeHigh() {
    elevator.setWantedState(ElevatorState.ALGAE_HIGH);
    arm.setWantedState(ArmState.HORIZONTAL);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
  }

  public void handleAlgaeLow() {
    elevator.setWantedState(ElevatorState.ALGAE_LOW);
    arm.setWantedState(ArmState.HORIZONTAL);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleNet() {
    elevator.setWantedState(ElevatorState.NET);
    if ((elevator.getElevatorPosition()) < Constants.inchesToMeters(50)) {
      arm.setWantedState(ArmState.NET);
    }
  }

  public void handleProcessor() {
    elevator.setWantedState(ElevatorState.PROCESSOR);
    arm.setWantedState(ArmState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleIntakingState() {
    straightenator.setWantedState(StraightenatorState.DEFAULT);
    intake.setWantedState(Intake.IntakeState.INTAKING);
    // if (straightenator.isFar() && !manipulator.hasCoral()) {
    // arm.setWantedState(ArmState.HANDOFF);
    // if (arm.isReadyForHandoff()) {
    // manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    // elevator.setWantedState(ElevatorState.HANDOFF_LOW);
    // }
    // }
  }

  public void handleOuttakeState() {
    intake.setWantedState(Intake.IntakeState.DEFAULT);
  }

  public void handleIdleState() {

  }

  @Override
  public void periodic() {
    setManipulatorHasCoral(manipulator.hasCoral());

    Logger.recordOutput("SuperStructure State", currentSuperState);
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}