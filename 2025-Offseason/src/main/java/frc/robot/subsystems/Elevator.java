package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.SetPoints.ElevatorPosition;
import frc.robot.subsystems.Manipulator.ArmItem;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
    private final double elevatorAcceleration = 1482542976.0;
    private final double elevatorCruiseVelocity = 449929104911.0;
    private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

    public enum ElevatorState {
        DEFAULT,
        ZERO,
        AUTO_L1,
        AUTO_L2,
        AUTO_L3,
        AUTO_L4,
        AUTO_SCORE_L3,
        AUTO_SCORE_MORE_L3,
        L1,
        L2,
        L3,
        L4,
        FEEDER_INTAKE,
        L2_ALGAE,
        L3_ALGAE,
        GROUND_CORAL_INTAKE,
        GROUND_ALGAE_INTAKE,
        PROCESSOR,
        SCORE_L1,
        SCORE_L2,
        AUTO_SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        NET,
        OVER,
        LOLLIPOP,
        PREHANDOFF,
        HANDOFF
    }

    private double idleTime;
    private boolean firstTimeIdle = true;
    private ElevatorState wantedState = ElevatorState.DEFAULT;
    private ElevatorState systemState = ElevatorState.DEFAULT;
    private double distanceFromL23DriveSetpoint = 0.0;
    private boolean firstTimeDefault = false;

    private ArmItem intakeItem = ArmItem.NONE;

    public void updateIntakeItem(ArmItem intakeItem) {
        this.intakeItem = intakeItem;
    }

    public void updateDistanceFromL23DriveSetpoint(double distanceFromL23DriveSetpoint) {
        this.distanceFromL23DriveSetpoint = distanceFromL23DriveSetpoint;
    }

    public double getElevatorL3ScoreSetpoint() {
        if (ElevatorPosition.kAUTOL3.meters - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.2 > 0.0) {
            return (ElevatorPosition.kAUTOL3.meters
                    - 3.5 / 39.37 - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.5);
        } else {
            return 0.0;
        }
    }

    public Elevator() {

    }

    public void teleopInit() {
        firstTimeIdle = true;
        firstTimeDefault = false;
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 60;
        currentLimitsConfigs.SupplyCurrentLimit = 60;
        elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
        elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
    }

    public void autoInit() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 60;
        currentLimitsConfigs.SupplyCurrentLimit = 60;
        elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
        elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
    }

    public void init() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        double elevatorMultiplier = 45.01 / 33.39;
        elevatorConfig.Slot0.kP = 33.39 * elevatorMultiplier;
        elevatorConfig.Slot0.kI = 0.0 * elevatorMultiplier;
        elevatorConfig.Slot0.kD = 2.7 * elevatorMultiplier;
        elevatorConfig.Slot0.kG = 4.499 * elevatorMultiplier;
        elevatorConfig.Slot1.kP = 75.83 * elevatorMultiplier;
        elevatorConfig.Slot1.kI = 0.0 * elevatorMultiplier;
        elevatorConfig.Slot1.kD = 4.690 * elevatorMultiplier;
        elevatorConfig.Slot1.kG = 8.044 * elevatorMultiplier;
        elevatorConfig.Slot2.kP = 33.39 * elevatorMultiplier * 0.5;
        elevatorConfig.Slot2.kI = 0.0 * elevatorMultiplier * 0.5;
        elevatorConfig.Slot2.kD = 2.7 * elevatorMultiplier * 0.5;
        elevatorConfig.Slot2.kG = 4.499 * elevatorMultiplier * 0.5;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = this.elevatorAcceleration;
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = this.elevatorCruiseVelocity;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        elevatorMotorMaster.getConfigurator().apply(elevatorConfig);
        elevatorMotorFollower.getConfigurator().apply(elevatorConfig);
        elevatorMotorMaster.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorFollower.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorMaster.setPosition(0.0);
        elevatorMotorFollower.setPosition(0.0);
    }

    public void moveWithPercent(double percent) {
        elevatorMotorMaster.set(percent);
        elevatorMotorFollower.set(-percent);
    }

    public void moveWithTorque(double current, double maxPercent) {
        elevatorMotorMaster.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
        elevatorMotorFollower.setControl(torqueCurrentFOCRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
    }

    public void moveElevatorToPosition(double position) {
        if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
            elevatorMotorMaster.setControl(
                    elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                            .withSlot(0));
            elevatorMotorFollower.setControl(
                    elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                            .withSlot(0));
        } else {
            if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
                moveWithTorque(18, 0.20);
                // System.out.println("running torque");
            } else {
                elevatorMotorMaster.setControl(
                        elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                                .withSlot(1));
                elevatorMotorFollower.setControl(
                        elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                                .withSlot(1));
            }
        }
    }

    public void moveElevatorToPositionSlow(double position) {
        if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
            elevatorMotorMaster.setControl(
                    elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                            .withSlot(2));
            elevatorMotorFollower.setControl(
                    elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                            .withSlot(2));
        } else {
            if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
                moveWithTorque(18, 0.20);
                // System.out.println("running torque");
            } else {
                elevatorMotorMaster.setControl(
                        elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                                .withSlot(1));
                elevatorMotorFollower.setControl(
                        elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                                .withSlot(1));
            }
        }
    }

    public double getElevatorPosition() {
        return Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getPosition().getValueAsDouble());
    }

    public void setElevatorEncoderPosition(double position) {
        elevatorMotorMaster.setPosition(position);
        elevatorMotorFollower.setPosition(position);
    }

    public void setWantedState(ElevatorState wantedState) {
        this.wantedState = wantedState;
    }

    private ElevatorState handleStateTransition() {
        switch (wantedState) {
            case DEFAULT:
                return ElevatorState.DEFAULT;
            case ZERO:
                return ElevatorState.ZERO;
            case OVER:
                return ElevatorState.OVER;
            case L1:
                return ElevatorState.L1;
            case L2:
                return ElevatorState.L2;
            case L3:
                return ElevatorState.L3;
            case L4:
                return ElevatorState.L4;
            case AUTO_L1:
                return ElevatorState.AUTO_L1;
            case AUTO_L2:
                return ElevatorState.AUTO_L2;
            case AUTO_L3:
                return ElevatorState.AUTO_L3;
            case AUTO_L4:
                return ElevatorState.AUTO_L4;
            case AUTO_SCORE_L3:
                return ElevatorState.AUTO_SCORE_L3;
            case AUTO_SCORE_MORE_L3:
                return ElevatorState.AUTO_SCORE_MORE_L3;
            case FEEDER_INTAKE:
                return ElevatorState.FEEDER_INTAKE;
            case L2_ALGAE:
                return ElevatorState.L2_ALGAE;
            case L3_ALGAE:
                return ElevatorState.L3_ALGAE;
            case GROUND_CORAL_INTAKE:
                return ElevatorState.GROUND_CORAL_INTAKE;
            case GROUND_ALGAE_INTAKE:
                return ElevatorState.GROUND_ALGAE_INTAKE;
            case PROCESSOR:
                return ElevatorState.PROCESSOR;
            case SCORE_L1:
                return ElevatorState.SCORE_L1;
            case SCORE_L2:
                return ElevatorState.SCORE_L2;
            case AUTO_SCORE_L2:
                return ElevatorState.AUTO_SCORE_L2;
            case SCORE_L3:
                return ElevatorState.SCORE_L3;
            case SCORE_L4:
                return ElevatorState.SCORE_L4;
            case NET:
                return ElevatorState.NET;
            case LOLLIPOP:
                return ElevatorState.LOLLIPOP;
            case HANDOFF:
                return ElevatorState.HANDOFF;
            case PREHANDOFF:
                return ElevatorState.PREHANDOFF;
            default:
                return ElevatorState.DEFAULT;
        }
    }

    public boolean getZeroed() {
        if (Math.abs(elevatorMotorMaster.getStatorCurrent().getValueAsDouble()) > 10.0
                && Math.abs(elevatorMotorMaster.getVelocity().getValueAsDouble()) < 5.0) {
            return true;
        } else {
            return false;
        }
    }

    private double zeroTime = 0.0;

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        if (systemState != ElevatorState.DEFAULT || OI.driverMenuButton.getAsBoolean()) {
            firstTimeDefault = false;
            idleTime = Timer.getFPGATimestamp();
            zeroTime = 0.0;
        }
        // System.out.println("Elevator Current: " +
        // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Elevator Current",
        // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Elevator Idle Time", idleTime);
        // Logger.recordOutput("First Time Idle", firstTimeIdle);
        Logger.recordOutput("Elevator State", systemState);
        // Logger.recordOutput("Elevator Velocity",
        // Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getVelocity().getValueAsDouble()));
        Logger.recordOutput("Elevator Height", getElevatorPosition() * 39.37);
        switch (systemState) {
            case GROUND_CORAL_INTAKE:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kGROUNDCORAL.meters);
                break;
            case ZERO:
                moveWithTorque(-40, 0.7);
                if (getZeroed()) {
                    setElevatorEncoderPosition(0.0);
                }
                break;
            case NET:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kNET.meters);
                break;
            case PROCESSOR:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kPROCESSOR.meters);
                break;
            case L2_ALGAE:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL2ALGAE.meters);
                break;
            case L3_ALGAE:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL3ALGAE.meters);
                break;
            case GROUND_ALGAE_INTAKE:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kGROUNDALGAE.meters);
                break;
            case OVER:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kOVER.meters);
                break;
            case L1:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL1.meters);
                break;
            case SCORE_L1:
                firstTimeIdle = true;
                break;
            case L2:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL2.meters);
                break;
            case AUTO_SCORE_L2:
                moveElevatorToPosition(ElevatorPosition.kAUTOL2SCORE.meters);
                break;
            case SCORE_L2:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL2.meters - 0.1);
                break;
            case L3:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL3.meters);
                break;
            case SCORE_L3:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL3.meters - 0.2);
                break;
            case L4:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL4.meters);
                break;
            case SCORE_L4:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL4.meters - 10 / 39.37);
                break;
            case FEEDER_INTAKE:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kFEEDER.meters);
                break;
            case AUTO_L1:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kL1.meters);
                break;
            case AUTO_L2:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kAUTOL2.meters);
                break;
            case AUTO_L3:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kAUTOL3.meters);
                break;
            case AUTO_L4:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kAUTOL4.meters);
                break;
            case AUTO_SCORE_MORE_L3:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kAUTOL3SCORE.meters);
                break;
            case AUTO_SCORE_L3:
                firstTimeIdle = true;
                moveElevatorToPosition(getElevatorL3ScoreSetpoint());
                break;
            case LOLLIPOP:
                firstTimeIdle = true;
                moveElevatorToPosition(ElevatorPosition.kLOLLIPOP.meters);
                break;
            case HANDOFF:
                firstTimeDefault = true;
                moveElevatorToPositionSlow(ElevatorPosition.kHANDOFF.meters);
                break;
            case PREHANDOFF:
                firstTimeDefault = true;
                moveElevatorToPosition(ElevatorPosition.kPREHANDOFF.meters);
                break;
            default:
                if (DriverStation.isTeleopEnabled()) {
                    // System.out.println("Stupid ahh ts pmo 1");
                    if (firstTimeIdle) {
                        // System.out.println("Stupid ahh ts pmo 2");
                        idleTime = Timer.getFPGATimestamp();
                        firstTimeIdle = false;
                    }
                    if (!firstTimeDefault || OI.driverMenuButton.getAsBoolean()) {
                        // System.out.println("Stupid ahh ts pmo 3");
                        if (DriverStation.isTeleopEnabled() && Math
                                .abs(
                                        Constants.Ratios
                                                .elevatorRotationsToMeters(
                                                        elevatorMotorMaster.getVelocity().getValueAsDouble())) < 0.1
                                && Timer.getFPGATimestamp() - idleTime > 0.3
                                && !firstTimeIdle) {
                            // System.out.println("Stupid ahh ts pmo 4");
                            if (zeroTime == 0.0) {
                                // System.out.println("Stupid ahh ts pmo 5");
                                zeroTime = Timer.getFPGATimestamp();
                            } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
                                // System.out.println("Stupid ahh ts pmo 6");
                                firstTimeDefault = true;
                                moveWithPercent(0.0);
                                setElevatorEncoderPosition(0.0);
                            }
                        } else {
                            // System.out.println("Stupid ahh ts pmo 7");
                            // System.out.println("Running down to zero");
                            if (intakeItem == ArmItem.ALGAE) {
                                // System.out.println("Stupid ahh ts pmo 8");

                                moveWithTorque(-40, 0.1);
                            } else {
                                // System.out.println("Stupid ahh ts pmo 9");
                                if (getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
                                    moveWithTorque(-50, 1.0);
                                    // System.out.println("Stupid ahh ts pmo 25");
                                } else if (getElevatorPosition() > (Constants.inchesToMeters(1.0))) {
                                    moveWithTorque(-30, 0.4);
                                    // System.out.println("Stupid ahh ts pmo 26");
                                }
                            }
                        }
                    } else {
                        // System.out.println("Stupid ahh ts pmo 10");
                        if (getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
                            moveWithTorque(-50, 1.0);
                            // System.out.println("Stupid ahh ts pmo 11");
                        } else if (getElevatorPosition() > (Constants.inchesToMeters(1.0))) {
                            moveWithTorque(-30, 0.4);
                            // System.out.println("Stupid ahh ts pmo 12");
                        } else {
                            // System.out.println("Stupid ahh ts pmo 13");
                            if (DriverStation.isTeleopEnabled() && Math
                                    .abs(
                                            Constants.Ratios
                                                    .elevatorRotationsToMeters(
                                                            elevatorMotorMaster.getVelocity().getValueAsDouble())) < 0.1
                                    && Timer.getFPGATimestamp() - idleTime > 0.3
                                    && !firstTimeIdle) {
                                // System.out.println("Stupid ahh ts pmo 14");
                                if (zeroTime == 0.0) {
                                    // System.out.println("Stupid ahh ts pmo 15");
                                    zeroTime = Timer.getFPGATimestamp();
                                } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
                                    // System.out.println("Stupid ahh ts pmo 16");
                                    firstTimeDefault = true;
                                    moveWithPercent(0.0);
                                    setElevatorEncoderPosition(0.0);
                                }
                            } else {
                                // System.out.println("Stupid ahh ts pmo 17");
                                // IntakeItem.ALGAE) {

                                if (intakeItem == ArmItem.ALGAE) {
                                    // System.out.println("Stupid ahh ts pmo 18");
                                    moveWithTorque(-40, 0.1);
                                } else {
                                    // System.out.println("Stupid ahh ts pmo 19");
                                    moveWithTorque(-40, 0.4);
                                }
                            }
                        }
                    }
                } else {
                    // System.out.println("Stupid ahh ts pmo 20");
                    if (Math.abs(
                            Constants.Ratios.elevatorRotationsToMeters(
                                    elevatorMotorMaster.getVelocity().getValueAsDouble())) < 0.1
                            && Timer.getFPGATimestamp() - idleTime > 0.3
                            && !firstTimeIdle) {
                        // System.out.println("Stupid ahh ts pmo 21");
                        if (zeroTime == 0.0) {
                            // System.out.println("Stupid ahh ts pmo 22");
                            zeroTime = Timer.getFPGATimestamp();
                        } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
                            // System.out.println("Stupid ahh ts pmo 23");
                            firstTimeDefault = true;
                            moveWithPercent(0.0);
                            setElevatorEncoderPosition(0.0);
                        }
                    } else {
                        // System.out.println("Running down to zero"); if (intakeItem ==
                        // IntakeItem.ALGAE) {
                        // System.out.println("Stupid ahh ts pmo 24");

                        moveWithTorque(-40, 0.6);

                    }
                }
                break;
        }
    }
}