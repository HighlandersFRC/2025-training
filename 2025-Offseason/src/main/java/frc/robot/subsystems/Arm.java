package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.controlloops.PID;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;
    private final PID armPID = new PID(0.03, 0.0, 0.005);

    private static final double ROT_NEG90_DEG = 0;
    private static final double ROT_POS90_DEG = -0.464;
    private static final double CAL_M = (90.0 - (-90.0)) / (ROT_POS90_DEG - ROT_NEG90_DEG);
    private static final double CAL_B = -90.0 - CAL_M * ROT_NEG90_DEG;

    public enum ArmState {
        DEFAULT,
        L4_SCORE,
        L4_PLACE,
        L3_SCORE,
        L3_PLACE,
        L2_SCORE,
        L2_PLACE,
        L1_PLACE,
        HORIZONTAL,
        VERTICAL,
        NET,
        HANDOFF,
        IDLE
    }

    private ArmState wantedState = ArmState.DEFAULT;
    private ArmState systemState = ArmState.DEFAULT;

    public Arm() {
        armMotor = new TalonFX(Constants.CANInfo.ARM_PIVOT_MOTOR_ID, new CANBus(Constants.CANInfo.CANBUS_NAME));
        // armMotor.setPosition(0);
    }

    public void zeroOnEnable() {
        armMotor.setPosition(0.0);
    }

    public void init() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;

        pivotConfig.Voltage.PeakForwardVoltage = 12.0;
        pivotConfig.Voltage.PeakReverseVoltage = -12.0;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.PIVOT_GEAR_RATIO;

        armMotor.getConfigurator().apply(pivotConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

        armPID.setMaxOutput(10.0);
        armPID.setMinOutput(-10.0);

        zeroOnEnable();
    }

    public double getArmRotations() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public double getRotorRotations() {
        return armMotor.getRotorPosition().getValueAsDouble();
    }

    public double getArmDegrees() {
        double rotations = getArmRotations();
        return CAL_M * rotations + CAL_B;
    }

    private double degreesToRotations(double degrees) {
        return (degrees - CAL_B) / CAL_M;
    }

    public boolean isReadyForHandoff() {
        return getArmDegrees() < -85.0;
    }

    public void setArmDegrees(double degrees) {
        armPID.setSetPoint(degrees);

        Logger.recordOutput("Arm Target Degrees", degrees);
        Logger.recordOutput("Arm Current Degrees", getArmDegrees());
    }

    public void setWantedState(ArmState state) {
        wantedState = state;
    }

    private ArmState handleStateTransition() {
        switch (wantedState) {
            case IDLE:
                return ArmState.IDLE;
            case L4_SCORE:
                return ArmState.L4_SCORE;
            case L4_PLACE:
                return ArmState.L4_PLACE;
            case L3_SCORE:
                return ArmState.L3_SCORE;
            case L3_PLACE:
                return ArmState.L3_PLACE;
            case L2_SCORE:
                return ArmState.L2_SCORE;
            case L2_PLACE:
                return ArmState.L2_PLACE;
            case L1_PLACE:
                return ArmState.L1_PLACE;
            case DEFAULT:
                return ArmState.DEFAULT;
            case HORIZONTAL:
                return ArmState.HORIZONTAL;
            case NET:
                return ArmState.NET;
            case HANDOFF:
                return ArmState.HANDOFF;
            case VERTICAL:
                return ArmState.VERTICAL;
            default:
                return wantedState;
        }
    }

    public void setCurrentLimit(int limit) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimit = limit;
        cfg.CurrentLimits.SupplyCurrentLimit = limit;
        armMotor.getConfigurator().apply(cfg);
        Logger.recordOutput("Arm Current Limit", limit);
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        switch (systemState) {
            case IDLE:
                armPID.setSetPoint(getArmDegrees());
                break;
            case L4_SCORE:
                setArmDegrees(Constants.Arm.L4_Score);
                break;
            case L4_PLACE:
                setArmDegrees(Constants.Arm.L4_Place);
                break;
            case L3_SCORE:
                setArmDegrees(Constants.Arm.L3_Score);
                break;
            case L3_PLACE:
                setArmDegrees(Constants.Arm.L3_Place);
                break;
            case L2_SCORE:
                setArmDegrees(Constants.Arm.L2_Score);
                break;
            case L2_PLACE:
                setArmDegrees(Constants.Arm.L2_Place);
                break;
            case L1_PLACE:
                setArmDegrees(Constants.Arm.L1_Place);
                break;
            case DEFAULT:
                setArmDegrees(Constants.Arm.DEFAULT);
                break;
            case HANDOFF:
                setArmDegrees(Constants.Arm.HANDOFF);
                break;
            case NET:
                setArmDegrees(Constants.Arm.NET);
                break;
            case HORIZONTAL:
                setArmDegrees(Constants.Arm.HORIZONTAL);
                break;
            case VERTICAL:
                setArmDegrees(Constants.Arm.VERTICAL);
                break;
        }

        double pidOutput = armPID.updatePID(getArmDegrees());
        armMotor.set(-pidOutput);

        Logger.recordOutput("Arm Rotations", getArmRotations());
        Logger.recordOutput("Arm RotorRotations", getRotorRotations());
        Logger.recordOutput("Arm Degrees", getArmDegrees());
        Logger.recordOutput("Arm Wanted State", systemState.toString());
        Logger.recordOutput("Arm Supply Current", armMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Arm Stator Current", armMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Arm Motor Voltage", armMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Arm Velocity", armMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Arm PID Output", pidOutput);
        Logger.recordOutput("Arm Ready For Handoff", isReadyForHandoff());
    }
}