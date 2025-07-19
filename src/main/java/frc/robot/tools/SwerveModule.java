package frc.robot.tools;

import frc.robot.Constants;
import frc.robot.tools.math.PID;
import frc.robot.tools.math.Vector;

import java.util.logging.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private static final double SPEED_DEADBAND = 1e-3;
    private final TalonFX motorDrive;
    private final TalonFX motorTurn;
    private final CANcoder motorEncoder;
    private final Vector turnVector;
    private final double wheelCircumference = Constants.Physical.WHEEL_CIRCUMFERENCE;
    private final PID turnPID = new PID(0.01, 0.0, 0.0);
    private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueFOCRequest = new VelocityTorqueCurrentFOC(0);
    private int moduleIndex;
    private double lastTargetRad = 0.0;

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder encoder, int index) {
        this.motorDrive = driveMotor;
        this.motorTurn = turnMotor;
        this.motorEncoder = encoder;
        this.turnVector = new Vector(0, 0);
        this.moduleIndex = index;

        double c = 1.0 / Math.sqrt(2);
        switch (index) {
            case 1:
                turnVector.setI(c);
                turnVector.setJ(c);
                break;
            case 2:
                turnVector.setI(-c);
                turnVector.setJ(c);
                break;
            case 3:
                turnVector.setI(-c);
                turnVector.setJ(-c);
                break;
            case 4:
                turnVector.setI(c);
                turnVector.setJ(-c);
                break;
        }

        turnPID.setContinuous(true);
        turnPID.setMinInput(0);
        turnPID.setMaxInput(360);

        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

        angleMotorConfig.Slot0.kP = 370.0;
        angleMotorConfig.Slot0.kI = 0.0;
        angleMotorConfig.Slot0.kD = 15;

        angleMotorConfig.Slot1.kP = 3.0;
        angleMotorConfig.Slot1.kI = 0.0;
        angleMotorConfig.Slot1.kD = 0.0;

        angleMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        angleMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -70;

        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        angleMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

        angleMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleMotorConfig.Feedback.FeedbackRemoteSensorID = motorEncoder.getDeviceID();
        angleMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleMotorConfig.Feedback.RotorToSensorRatio = Constants.Ratios.STEER_GEAR_RATIO;

        driveMotorConfig.Slot0.kP = 8.0;
        driveMotorConfig.Slot0.kI = 0.0;
        driveMotorConfig.Slot0.kD = 0.0;
        driveMotorConfig.Slot0.kV = 0.0;

        driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 120;

        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

        if (index == 4) {
            driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        double absolutePosition = motorEncoder.getAbsolutePosition().getValueAsDouble();
        motorTurn.setPosition(absolutePosition);
        motorDrive.setPosition(0.0);

        motorTurn.getConfigurator().apply(angleMotorConfig);
        motorDrive.getConfigurator().apply(driveMotorConfig);

    }

    public double findClosestAngle(double angleA, double angleB) {
        double diff = angleB - angleA;
        return (diff + Math.PI) % (2 * Math.PI) - Math.PI;
    }

    public void drive(Vector driveVector, double turnInput) {
        double vx = driveVector.getI() + turnInput * turnVector.getI();
        double vy = driveVector.getJ() + turnInput * turnVector.getJ();
        double speed = Math.hypot(vx, vy);
        double targetRad;
        if (speed > SPEED_DEADBAND) {
            double deg = Math.toDegrees(Math.atan2(vy, vx));
            if (deg < 0)
                deg += 360;
            double desiredRad = Math.toRadians(deg);
            double currentAbsRad = motorEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;

            double noFlip = findClosestAngle(currentAbsRad, desiredRad);
            double flip = findClosestAngle(currentAbsRad, desiredRad + Math.PI);

            if (Math.abs(flip) < Math.abs(noFlip)) {
                targetRad = currentAbsRad + flip;
                speed = -speed;
            } else {
                targetRad = currentAbsRad + noFlip;
            }

            lastTargetRad = targetRad;
        } else {
            targetRad = lastTargetRad;
            speed = 0;
        }

        setSpeed(speed * Constants.Physical.TOP_SPEED);

        double desiredRevs = targetRad / (2 * Math.PI);
        double currentMotorRevs = motorTurn.getPosition().getValueAsDouble();

        double diffRevs = findClosestAngle(
                motorEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                desiredRevs * 2 * Math.PI) / (2 * Math.PI);

        double targetRevs = currentMotorRevs + diffRevs;
        motorTurn.setControl(positionTorqueFOCRequest.withPosition(targetRevs));
    }

    private void setSpeed(double velocity) {
        motorDrive.setControl(velocityTorqueFOCRequest.withVelocity(wheelToDriveMotorRotations(velocity)));
    }

    public double wheelToSteerMotorRotations(double rotations) {
        return rotations * Constants.Ratios.STEER_GEAR_RATIO;
    }

    public double steerMotorToWheelRotations(double rotations) {
        return rotations / Constants.Ratios.STEER_GEAR_RATIO;
    }

    public double wheelToDriveMotorRotations(double rotations) {
        return rotations * Constants.Ratios.DRIVE_GEAR_RATIO;
    }

    public double driveMotorToWheelRotations(double rotations) {
        return rotations / Constants.Ratios.DRIVE_GEAR_RATIO;
    }

    public double degreesToRotations(double degrees) {
        return degrees / 360;
    }

    public double rotationsToDegrees(double rotations) {
        return rotations * 360;
    }

    public double getEncoder() {
        return motorEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getAngle() {
        double rotations = getEncoder();
        double angle = (rotations * 360) % 360;
        return angle < 0 ? angle + 360 : angle;
    }

    public void stop() {
        motorDrive.set(0);
        motorTurn.stopMotor();
        turnPID.setSetPoint(getAngle());
    }

    public void periodic() {
    }

    public SwerveModulePosition getPosition() {
        double motorRotations = motorDrive.getRotorPosition().getValueAsDouble();
        double wheelRotations = motorRotations / Constants.Ratios.DRIVE_GEAR_RATIO;
        double distanceMeters = wheelRotations * wheelCircumference;
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());
        return new SwerveModulePosition(distanceMeters, angle);
    }
}
