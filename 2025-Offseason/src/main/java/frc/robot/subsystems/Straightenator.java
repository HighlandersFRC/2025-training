package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class Straightenator extends SubsystemBase {
    private final TalonFX left_straightenator = new TalonFX(Constants.CANInfo.LEFT_STRAIGHTENATOR_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX right_straightenator = new TalonFX(Constants.CANInfo.RIGHT_STRAIGHTENATOR_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);

    private final DigitalInput closeBeamBreak = new DigitalInput(Constants.CANInfo.CLOSE_BEAM_BREAK_SENSOR);
    private final DigitalInput farBeamBreak = new DigitalInput(Constants.CANInfo.FAR_BEAM_BREAK_SENSOR);

    private StraightenatorState wantedState = StraightenatorState.DEFAULT;
    private StraightenatorState systemState = StraightenatorState.DEFAULT;

    private final double voltageThreshold = 40;
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    public Straightenator() {
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 60;
        left_straightenator.getConfigurator().apply(leftConfig);
        right_straightenator.getConfigurator().apply(rightConfig);
    }

    public double timeStalled = 0;
    public boolean isStalling = false;

    public enum StraightenatorState {
        DEFAULT,
        INTAKE,
        INTAKE_LEFT,
        INTAKE_RIGHT,
        OUTTAKE_ONESIDE,
        OUTTAKE,
        IDLE
    }

    public boolean isReady() {
        return isClose() && isFar();
    }

    public boolean isClose() {
        return !closeBeamBreak.get();
    }

    public boolean isFar() {
        return !farBeamBreak.get();
    }

    public double getLeftVoltage() {
        return left_straightenator.getMotorVoltage().getValueAsDouble();
    }

    public double getRightVoltage() {
        return right_straightenator.getMotorVoltage().getValueAsDouble();
    }

    public void moveWithTorque(double current, double maxPercent) {
        left_straightenator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
        right_straightenator.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    }

    public void moveWithTorque(double leftCurrent, double leftMaxPercent, double rightCurrent, double rightMaxPercent) {
        left_straightenator
                .setControl(torqueCurrentFOCRequest.withOutput(leftCurrent).withMaxAbsDutyCycle(leftMaxPercent));
        right_straightenator
                .setControl(torqueCurrentFOCRequest.withOutput(rightCurrent).withMaxAbsDutyCycle(rightMaxPercent));
    }

    public StraightenatorState handleStateTransition() {
        switch (wantedState) {
            case IDLE:
                return StraightenatorState.IDLE;
            case DEFAULT:
                return StraightenatorState.DEFAULT;
            case INTAKE:
                // double leftTorque =
                // left_straightenator.getTorqueCurrent().getValueAsDouble();
                // double rightTorque =
                // right_straightenator.getTorqueCurrent().getValueAsDouble();
                // if (Math.abs(leftTorque) > 50 || Math.abs(rightTorque) > 50) {
                // return StraightenatorState.INTAKE_LEFT;
                // } else
                if (isStalling) {
                    return StraightenatorState.INTAKE_LEFT;
                } else
                    return StraightenatorState.INTAKE;
            case INTAKE_LEFT:
                return StraightenatorState.INTAKE_LEFT;
            case INTAKE_RIGHT:
                return StraightenatorState.INTAKE_RIGHT;
            case OUTTAKE:
                return StraightenatorState.OUTTAKE;
            case OUTTAKE_ONESIDE:
                return StraightenatorState.OUTTAKE_ONESIDE;
            default:
                return StraightenatorState.DEFAULT;
        }
    }

    public void moveWithPercent(double left, double right) {
        left_straightenator.set(left);
        right_straightenator.set(right);
    }

    public void setWantedState(StraightenatorState state) {
        this.wantedState = state;
    }

    public void checkForStalling() {
        if (Math.abs(getLeftVoltage()) > voltageThreshold || Math.abs(getRightVoltage()) > voltageThreshold) {
            if (!isStalling) {
                timeStalled = Timer.getFPGATimestamp();
                isStalling = true;
            } else {
                if (Timer.getFPGATimestamp() - timeStalled > 0.5) {
                    isStalling = true;
                }
            }
        } else {
            isStalling = false;
            timeStalled = 0;
        }
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        if (OI.driverLT.getAsBoolean()) {
            systemState = StraightenatorState.OUTTAKE;
        } else
            switch (systemState) {
                case IDLE:
                    left_straightenator.set(0);
                    right_straightenator.set(0);
                    break;
                case DEFAULT:
                    moveWithTorque(10, 0.1);
                    if (isClose() && isFar()) {
                        moveWithTorque(0, 0);
                    }
                    break;
                case INTAKE_LEFT:
                    moveWithPercent(0.4, -0.4);
                    break;
                case INTAKE_RIGHT:
                    if (isClose() && isFar()) {
                        moveWithTorque(0, 0);
                    } else if (isClose()) {
                        moveWithTorque(10, 0.05, 20, 0.1);
                    } else {
                        moveWithTorque(20, 0.4, 40, 0.75);
                    }
                    break;
                case INTAKE:
                    // moveWithTorque(20, 0.4, 20, 0.4);
                    moveWithPercent(0.2, 0.4);
                    break;
                case OUTTAKE:
                    moveWithTorque(-20, 0.3);
                case OUTTAKE_ONESIDE:
                    moveWithPercent(-0.4, 0);
                default:
                    break;
            }
        checkForStalling();
        org.littletonrobotics.junction.Logger.recordOutput("Left Straightenator Voltage",
                left_straightenator.getTorqueCurrent().getValueAsDouble());
        org.littletonrobotics.junction.Logger.recordOutput("Right Straightenator Voltage",
                right_straightenator.getTorqueCurrent().getValueAsDouble());
        org.littletonrobotics.junction.Logger.recordOutput("Left Straightenator Stator Current",
                left_straightenator.getStatorCurrent().getValueAsDouble());
        org.littletonrobotics.junction.Logger.recordOutput("Right Straightenator Stator Current",
                right_straightenator.getStatorCurrent().getValueAsDouble());
        org.littletonrobotics.junction.Logger.recordOutput("Close Beam Break", closeBeamBreak.get());
        org.littletonrobotics.junction.Logger.recordOutput("Far Beam Break", farBeamBreak.get());
        org.littletonrobotics.junction.Logger.recordOutput("Straightenator State", systemState);
    }
}