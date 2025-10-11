// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.IntFunction;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Straightenator.StraightenatorState;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum SuperState {
    DEFAULT,
    ZERO,
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
    INTAKE_IDLE,
    OUTTAKE_ONESIDE,
    GROUND_ALGAE_INTAKE,
    PICKUP_PIECE,
    IDLE,
    OUTAKE_IDLE
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
  Pivot pivot;
  Manipulator manipulator;
  Intake intake;

  public enum CurrentMode {
    ALGAE,
    CORAL
  }

  public CurrentMode currentMode = CurrentMode.CORAL;

  public Superstructure(Drive drive, Elevator elevator, Manipulator manipulator, Pivot pivot, Peripherals peripheral,
      Intake intake, Straightenator straightenator) {
    this.drive = drive;
    this.peripherals = peripheral;
    this.elevator = elevator;
    this.straightenator = straightenator;
    this.pivot = pivot;
    this.manipulator = manipulator;
    this.intake = intake;
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
    System.out.println("Wanted State: " + wantedState);
    this.wantedSuperState = wantedState;

  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean isPathCompleted() {
    return pathCompleted;
  }

  private double backUpTime = Timer.getFPGATimestamp();

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        drive.setWantedState(DriveState.DEFAULT);
        handleDefaultState();
        break;
      case ZERO:
        handleZeroState();
        break;
      case MANIPULATOR_OUTTAKE:
        manipulator.setWantedState(ManipulatorState.OUTAKE);
        break;
      case OUTTAKE:
        handleOuttakeState();
        break;
      case GROUND_ALGAE_INTAKE:
        handleGroundAlgaeIntake();
        break;
      case AUTO_L1_SCORE:
        handleAutoL1Score();
        break;
      case AUTO_L2_PLACE:
        System.out.println("Auto L2 Place");
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
        if (straightenator.isFar() && !manipulator.hasCoralSemiSticky()) {
          handleHandOffLowState();
          // if (Constants.metersToInches(elevator.getElevatorPosition()) <
          // Constants.Elevator.HANDOFF_LOW + 2.0) {
          // handleHandoffState();
          // }
        } else {
          handleHandoffState();
        }

        // if (!manipulator.hasCoral()) {
        // if (straightenator.isFar()) {
        // handleHandOffLowState();
        // } else if (Constants.metersToInches(elevator.getElevatorPosition()) <
        // Constants.Elevator.HANDOFF_LOW + 2.0) {
        // handleHandoffState();
        // }
        // } else {
        // handleHandoffState();
        // }

        break;
      default:
        handleHandoffState();
        break;
      case IDLE:
        handleIdleState();
        break;
      case OUTAKE_IDLE:
        handleOutakeIdleState();
        break;
      case MOVE_TO_POINT:
        // drive.setWantedState(DriveState.MOVE_TO_POINT);
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
      case INTAKE_IDLE:

        if (straightenator.isFar() && !manipulator.hasCoral()) {
          elevator.setWantedState(ElevatorState.HANDOFF_LOW);
          manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
          // if (Constants.metersToInches(elevator.getElevatorPosition()) <
          // Constants.Elevator.HANDOFF_LOW + 2.0) {
          // handleHandoffState();
          // }
        } else {
          elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
          manipulator.setWantedState(ManipulatorState.DEFAULT);
        }
        if (straightenator.isFar() || manipulator.hasCoral()) {
          straightenator.setWantedState(StraightenatorState.IDLE);
        } else {
          straightenator.setWantedState(StraightenatorState.INTAKE);
        }
        intake.setWantedState(IntakeState.IDLE);
        break;
      case OUTTAKE_ONESIDE:
        straightenator.setWantedState(Straightenator.StraightenatorState.OUTTAKE_ONESIDE);
        break;
    }
  }

  private double zeroTime = Timer.getFPGATimestamp();
  private boolean firstTimeZero = true;

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case ZERO:
        if (intake.getZeroed() && elevator.getZeroed() && pivot.getZeroed() && firstTimeZero) {
          firstTimeZero = false;
          zeroTime = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - zeroTime > 1.0 && intake.getZeroed() && elevator.getZeroed()) {
          currentSuperState = SuperState.DEFAULT;
          wantedSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.ZERO;
        }
        break;
      case MANIPULATOR_OUTTAKE:
        currentSuperState = SuperState.MANIPULATOR_OUTTAKE;
        break;
      case OUTTAKE:
        currentSuperState = SuperState.OUTTAKE;
        break;
      case AUTO_L2_PLACE:
        Pose2d closestl2 = drive.getReefL3ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA());
        java.util.logging.Logger.getGlobal().finer(
            "Drive: " + drive.hitSetPoint(closestl2));
        java.util.logging.Logger.getGlobal().finer(
            "Elevator: "
                + (elevator.getElevatorPosition() > Constants.inchesToMeters(Constants.Elevator.AUTO_SCORE_L2) - 5));
        if ((drive.hitSetPoint(closestl2))
            && elevator.getElevatorPosition() > Constants
                .inchesToMeters(Constants.Elevator.AUTO_SCORE_L2 - 5)
            || OI.getDriverLB()) {
          currentSuperState = SuperState.AUTO_L2_SCORE;
          wantedSuperState = SuperState.AUTO_L2_SCORE;
          backUpTime = Timer.getFPGATimestamp();
        } else {
          currentSuperState = SuperState.AUTO_L2_PLACE;
          backUpTime = Timer.getFPGATimestamp();
        }
        break;
      case AUTO_L2_SCORE:
        if (Timer.getFPGATimestamp() - backUpTime > 0.5) {
          wantedSuperState = SuperState.DEFAULT;
          currentSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.AUTO_L2_SCORE;
        }
        break;
      case AUTO_L3_PLACE:
        Pose2d closestl3 = drive.getReefL3ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA());
        java.util.logging.Logger.getGlobal().finer(
            "Drive: " + drive.hitSetPoint(closestl3));
        java.util.logging.Logger.getGlobal().finer(
            "Elevator: "
                + (elevator.getElevatorPosition() > Constants.inchesToMeters(Constants.Elevator.AUTO_SCORE_L3) - 3));
        if ((drive.hitSetPoint(closestl3))
            && elevator.getElevatorPosition() > Constants
                .inchesToMeters(Constants.Elevator.AUTO_SCORE_L3 - 3)
            || OI.getDriverLB()) {
          currentSuperState = SuperState.AUTO_L3_SCORE;
          wantedSuperState = SuperState.AUTO_L3_SCORE;
          backUpTime = Timer.getFPGATimestamp();
        } else {
          currentSuperState = SuperState.AUTO_L3_PLACE;
          backUpTime = Timer.getFPGATimestamp();
        }
        break;
      case AUTO_L3_SCORE:
        if (Timer.getFPGATimestamp() - backUpTime > 0.5) {
          wantedSuperState = SuperState.DEFAULT;
          currentSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.AUTO_L3_SCORE;
        }
        break;
      case AUTO_L4_SCORE:
        if (Timer.getFPGATimestamp() - backUpTime > 0.5) {
          wantedSuperState = SuperState.DEFAULT;
          currentSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.AUTO_L4_SCORE;
        }
        break;
      case HANDOFF:
        currentSuperState = SuperState.HANDOFF;
        break;
      case AUTO_L4_PLACE:
        Pose2d closestl4 = drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA());
        java.util.logging.Logger.getGlobal().finer(
            "Drive: " + drive.hitSetPoint(closestl4));
        java.util.logging.Logger.getGlobal().finer(
            "Elevator: " + (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL4.meters));
        if ((drive.hitSetPoint(closestl4))
            && elevator.getElevatorPosition() > Constants
                .metersToInches(Constants.Elevator.AUTO_SCORE_L4 - 5)
            || OI.getDriverLB()) {
          currentSuperState = SuperState.AUTO_L4_SCORE;
          wantedSuperState = SuperState.AUTO_L4_SCORE;
          backUpTime = Timer.getFPGATimestamp();
        } else {
          currentSuperState = SuperState.AUTO_L4_PLACE;
          backUpTime = Timer.getFPGATimestamp();
        }
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
        if (straightenator.isFar() && !manipulator.hasCoralSemiSticky()) {
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
      case GROUND_ALGAE_INTAKE:
        currentSuperState = SuperState.GROUND_ALGAE_INTAKE;
        break;
      case ALGAE_HOME:
        currentSuperState = SuperState.ALGAE_HOME;
        break;
      case INTAKE_IDLE:
        currentSuperState = SuperState.INTAKE_IDLE;
        break;
      case OUTTAKE_ONESIDE:
        currentSuperState = SuperState.OUTTAKE_ONESIDE;
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

  public void handleZeroState() {
    elevator.setWantedState(ElevatorState.ZERO);
    intake.setWantedState(IntakeState.ZERO);
    pivot.setWantedState(PivotState.ZERO);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (DriverStation.isAutonomousEnabled()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleAlgaeHome() {
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    pivot.setWantedState(PivotState.VERTICAL);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    intake.setWantedState(IntakeState.DEFAULT);
  }

  public void handleHandoffState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    pivot.setWantedState(PivotState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
  }

  public void handleHandOffLowState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.HANDOFF_LOW);
    pivot.setWantedState(PivotState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    intake.setWantedState(IntakeState.DOWN);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
  }

  // public void handleDefaultState() {
  // drive.setWantedState(DriveState.DEFAULT);
  // elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
  // pivot.setWantedState(PivotState.HANDOFF);
  // intake.setWantedState(Intake.IntakeState.DEFAULT);
  // manipulator.setWantedState(ManipulatorState.OFF);
  // }

  public void handleAutoL1Score() {
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    elevator.setWantedState(ElevatorState.AUTO_L1);
    intake.setWantedState(IntakeState.DOWN);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (elevator.getElevatorPosition() < Constants.inchesToMeters(9.0)) {
      pivot.setWantedState(PivotState.HANDOFF);
    } else {
      pivot.setWantedState(PivotState.L1_PLACE);
    }
  }

  public void handleAutoL2Place() {
    pivot.setWantedState(PivotState.L2_PLACE);
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_L2);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (elevator.getElevatorPosition() > Constants
        .inchesToMeters(Constants.Elevator.AUTO_SCORE_L2 - 3)
        && pivot.getPivotDegrees() > Constants.Pivot.L2_Place - 4) {
      drive.setWantedState(DriveState.L3_REEF);
    }

  }

  public void handleAutoL2Score() {
    pivot.setWantedState(PivotState.L2_SCORE);
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L2);
    // if (Timer.getFPGATimestamp() - backUpTime > 0.5) {
    // manipulator.setWantedState(ManipulatorState.OUTAKE);
    // }
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (pivot.getPivotDegrees() < Constants.Pivot.HORIZONTAL + 2) {
      drive.setWantedState(DriveState.REEF_MORE);
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else
      drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleAutoL3Place() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_L3);
    pivot.setWantedState(PivotState.L3_PLACE);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (elevator.getElevatorPosition() > Constants
        .metersToInches(Constants.Elevator.AUTO_SCORE_L3 - 3)
        && pivot.getPivotDegrees() > Constants.Pivot.L3_Place - 4) {
      drive.setWantedState(DriveState.L3_REEF);
    }
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleAutoL3Score() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L3);
    pivot.setWantedState(PivotState.L3_SCORE);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (pivot.getPivotDegrees() < Constants.Pivot.HORIZONTAL + 2) {
      drive.setWantedState(DriveState.REEF_MORE);
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else
      drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleAutoL4Place() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_L4);
    pivot.setWantedState(PivotState.L4_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (elevator.getElevatorPosition() > Constants
        .metersToInches(Constants.Elevator.AUTO_SCORE_L4 - 5) && pivot.getPivotDegrees() > 20) {

      drive.setWantedState(DriveState.L4_REEF);
    }

  }

  public void handleAutoL4Score() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L4);
    pivot.setWantedState(PivotState.HORIZONTAL);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    if (pivot.getPivotDegrees() < Constants.Pivot.HORIZONTAL + 2) {
      drive.setWantedState(DriveState.REEF_MORE);
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else
      drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleL1Score() {
    elevator.setWantedState(ElevatorState.AUTO_L1);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    intake.setWantedState(IntakeState.DOWN);
    if (elevator.getElevatorPosition() < Constants.inchesToMeters(10.0)) {
      pivot.setWantedState(PivotState.HANDOFF);
    } else {
      pivot.setWantedState(PivotState.L1_PLACE);
    }
  }

  public void handleL2Place() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_L2);
    pivot.setWantedState(PivotState.L2_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleL2Score() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L2);
    pivot.setWantedState(PivotState.L2_SCORE);
  }

  public void handleL3Place() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_L3);
    pivot.setWantedState(PivotState.L3_PLACE);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleL3Score() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_L3);
    pivot.setWantedState(PivotState.L3_SCORE);
  }

  public void handleL4Place() {
    // intake.setWantedState(IntakeState.DOWN);
    // elevator.setWantedState(ElevatorState.AUTO_L4);
    // pivot.setWantedState(PivotState.L4_PLACE);
    // manipulator.setWantedState(ManipulatorState.DEFAULT);

  }

  public void handleL4Score() {
    // intake.setWantedState(IntakeState.DOWN);
    // elevator.setWantedState(ElevatorState.AUTO_SCORE_L4);
    // pivot.setWantedState(PivotState.HORIZONTAL);

  }

  public void handleAlgaeHigh() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.ALGAE_HIGH);
    pivot.setWantedState(PivotState.HORIZONTAL);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
  }

  public void handleAlgaeLow() {
    intake.setWantedState(IntakeState.DOWN);
    elevator.setWantedState(ElevatorState.ALGAE_LOW);
    pivot.setWantedState(PivotState.HORIZONTAL);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
  }

  public void handleNet() {
    elevator.setWantedState(ElevatorState.NET);
    if ((elevator.getElevatorPosition()) < Constants.inchesToMeters(50)) {
      pivot.setWantedState(PivotState.NET);
    }
  }

  public void handleProcessor() {
    elevator.setWantedState(ElevatorState.PROCESSOR);
    pivot.setWantedState(PivotState.PROCESSOR);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
  }

  public void handleGroundAlgaeIntake() {
    elevator.setWantedState(ElevatorState.GROUND_ALGAE_INTAKE);
    pivot.setWantedState(PivotState.GROUND_ALGAE_INTAKE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
  }

  public void handleIntakingState() {
    if (straightenator.isFar() || manipulator.hasCoral()) {
      straightenator.setWantedState(StraightenatorState.IDLE);
    } else {
      straightenator.setWantedState(StraightenatorState.INTAKE);
    }
    if (straightenator.isClose() || manipulator.hasCoral()) {
      intake.setWantedState(IntakeState.DOWN);
    } else {
      intake.setWantedState(IntakeState.INTAKING);
    }

    if (straightenator.isFar() && !manipulator.hasCoral()) {
      System.out.println("333");
      elevator.setWantedState(ElevatorState.HANDOFF_LOW);
      manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
      // if (Constants.metersToInches(elevator.getElevatorPosition()) <
      // Constants.Elevator.HANDOFF_LOW + 2.0) {
      // handleHandoffState();
      // }
    } else {
      System.out.println("4444444");
      elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
      manipulator.setWantedState(ManipulatorState.DEFAULT);
    }
    // if (straightenator.isFar() && !manipulator.hasCoral()) {
    // pivot.setWantedState(PivotState.HANDOFF);
    // if (pivot.isReadyForHandoff()) {
    // manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    // elevator.setWantedState(ElevatorState.HANDOFF_LOW);
    // }
    // }
  }

  public void handleOuttakeState() {
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
    intake.setWantedState(Intake.IntakeState.OUTTAKING);
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    pivot.setWantedState(PivotState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
  }

  public void handleOutakeIdleState() {
    drive.setWantedState(DriveState.IDLE);
    elevator.setWantedState(ElevatorState.HANDOFF_HIGH);
    pivot.setWantedState(PivotState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    intake.setWantedState(IntakeState.DOWN);
    straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
  }

  public boolean hasCoral() {
    return manipulator.hasCoral();
  }

  public boolean placedCoralL4() {
    // return
    // drive.hitSetPoint(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry()).getRotation().getRadians())
    // &&
    // elevator.getElevatorPosition() > 53 / 39.37
    // &&

    // Pivot has abs to account for placing backwards
    // double[] setpoint = drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(),
    // false);
    Pose2d setpoint = drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), false);
    Logger.recordOutput("auto l4 setpoint", setpoint);
    java.util.logging.Logger.getGlobal().fine((Math
        .abs(Math.abs(pivot.getPivotPosition())
            - Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations) < (10.0 / 360.0)
        && drive.hitSetPointGenerous(
            setpoint))
        + "");
    return (Math
        .abs(Math.abs(pivot.getPivotPosition())
            - Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations) < (10.0 / 360.0)
        && drive.hitSetPointGenerous(
            setpoint));
  }

  public boolean placedCoralL2() {
    // return
    // drive.hitSetPoint(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry()).getRotation().getRadians())
    // &&
    // elevator.getElevatorPosition() > 53 / 39.37
    // &&

    // Pivot has abs to account for placing backwards
    return Math
        .abs(Math.abs(pivot.getPivotPosition())
            - Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations) < (5.0 / 360.0);
  }

  @Override
  public void periodic() {
    setManipulatorHasCoral(manipulator.hasCoral());

    Logger.recordOutput("SuperStructure State", currentSuperState);
    // System.out
    // .println("Zero Intake: " + intake.getZeroed() + " Zero Elevator: " +
    // elevator.getZeroed() + " Pivot Zeroed: "
    // + pivot.getZeroed());
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}