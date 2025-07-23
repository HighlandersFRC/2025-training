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
    private static final double minSpeed = 1e-3;
    private final TalonFX motorDrive;
    private final TalonFX motorTurn;
    private final CANcoder motorEncoder;
    private final Vector turnVector;
    private final double wheelCircumference = Constants.Physical.WHEEL_CIRCUMFERENCE;
    private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueFOCRequest = new VelocityTorqueCurrentFOC(0);
    private int moduleIndex;
    private double lastTargetRad = 0.0;
    private double moduleOffsetX;
    private double moduleOffsetY;
    private double offset = 0.6096 / 2;

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder encoder, int index) {
        this.motorDrive = driveMotor;
        this.motorTurn = turnMotor;
        this.motorEncoder = encoder;
        this.turnVector = new Vector(0, 0);
        this.moduleIndex = index;

        double c = 1.0 / Math.sqrt(2);
        switch (index) {
            case 1:
                moduleOffsetX = offset;
                moduleOffsetY = offset;
                turnVector.setI(c);
                turnVector.setJ(c);
                break;
            case 2:
                moduleOffsetX = -offset;
                moduleOffsetY = offset;
                turnVector.setI(-c);
                turnVector.setJ(c);
                break;
            case 3:
                moduleOffsetX = -offset;
                moduleOffsetY = -offset;
                turnVector.setI(-c);
                turnVector.setJ(-c);
                break;
            case 4:
                moduleOffsetX = offset;
                moduleOffsetY = -offset;
                turnVector.setI(c);
                turnVector.setJ(-c);
                break;
        }

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

        if (index == 2 || index == 3) {
            driveMotorConfig.Slot0.kP = 9.4;
            driveMotorConfig.Slot0.kI = 0.0;
            driveMotorConfig.Slot0.kD = 0.0;
            driveMotorConfig.Slot0.kV = 0.0;
        } else {
            driveMotorConfig.Slot0.kP = 8.0;
            driveMotorConfig.Slot0.kI = 0.0;
            driveMotorConfig.Slot0.kD = 0.0;
            driveMotorConfig.Slot0.kV = 0.0;
        }

        driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 120;
        driveMotorConfig.Feedback.RotorToSensorRatio = 1;
        driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.DRIVE_GEAR_RATIO;

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

    public void drive(Vector driveVector, double turnInput, double navxAngle) {
        double vx = driveVector.getI();
        double vy = driveVector.getJ();

        double rotationalVx = turnInput * turnVector.getI();
        double rotationalVy = turnInput * turnVector.getJ();

        double totalVx = vx + rotationalVx;
        double totalVy = vy + rotationalVy;

        double speed = Math.hypot(totalVx, totalVy);
        double angle = Math.atan2(totalVy, totalVx);

        double absAngleRad = motorEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        double targetRad;

        if (speed > minSpeed) {
            while (angle < 0)
                angle += 2 * Math.PI;
            while (angle >= 2 * Math.PI)
                angle -= 2 * Math.PI;

            double noFlip = findClosestAngle(absAngleRad, angle);
            double flip = findClosestAngle(absAngleRad, angle + Math.PI);

            if (Math.abs(flip) < Math.abs(noFlip)) {
                targetRad = absAngleRad + flip;
                speed = -speed;
            } else {
                targetRad = absAngleRad + noFlip;
            }

            lastTargetRad = targetRad;
        } else {
            targetRad = lastTargetRad;
            speed = 0;
        }

        setSpeed(speed * Constants.Physical.TOP_SPEED);
        org.littletonrobotics.junction.Logger.recordOutput("Speed", speed * Constants.Physical.TOP_SPEED);
        org.littletonrobotics.junction.Logger.recordOutput("Current Speed",
                motorDrive.getVelocity().getValueAsDouble());
        double currentRevs = motorTurn.getPosition().getValueAsDouble();
        double diffRad = findClosestAngle(absAngleRad, targetRad);
        double deltaRevs = diffRad / (2 * Math.PI);

        final double MAX_DELTA_REVS = 0.5;
        if (Math.abs(deltaRevs) > MAX_DELTA_REVS) {
            deltaRevs = Math.signum(deltaRevs) * MAX_DELTA_REVS;
        }

        motorTurn.setControl(positionTorqueFOCRequest.withPosition(currentRevs + deltaRevs));
    }

    private double findClosestAngle(double currentAngle, double targetAngle) {
        double diff = targetAngle - currentAngle;

        while (diff > Math.PI)
            diff -= 2 * Math.PI;
        while (diff < -Math.PI)
            diff += 2 * Math.PI;

        return diff;
    }

    // public void drive(Vector vector, double turnValue, double navxAngle) {
    // if (Math.abs(vector.getI()) < 0.001 && Math.abs(vector.getJ()) < 0.001 &&
    // Math.abs(turnValue) < 0.01) {
    // // stop both motors
    // motorDrive.setControl(velocityTorqueFOCRequest.withVelocity(0.0));
    // motorTurn.setControl(velocityTorqueFOCRequest.withVelocity(0.0));
    // } else {
    // // desired translation
    // double angleWanted = Math.atan2(vector.getJ(), vector.getI());
    // double wheelPower = Math.hypot(vector.getI(), vector.getJ());

    // // field-centric adjustment
    // double angleWithNavx = angleWanted + navxAngle;
    // double xWithNavx = wheelPower * Math.cos(angleWithNavx);
    // double yWithNavx = wheelPower * Math.sin(angleWithNavx);

    // // rotation component (torqueAngle assumed to be current wheel angle)
    // double torqueAng = getEncoder();
    // double turnX = turnValue * Constants.angleToUnitVectorI(torqueAng);
    // double turnY = turnValue * Constants.angleToUnitVectorJ(torqueAng);

    // // combine translation + rotation
    // Vector finalVector = new Vector(xWithNavx + turnX, yWithNavx + turnY);

    // double finalAngle = -Math.atan2(finalVector.getJ(), finalVector.getI());
    // double finalVelocity = Math.hypot(finalVector.getI(), finalVector.getJ());

    // // cap speed
    // if (finalVelocity > Constants.Physical.TOP_SPEED) {
    // finalVelocity = Constants.Physical.TOP_SPEED;
    // }

    // double velocityRPS = MPSToRPS(finalVelocity);

    // // optimize wheel movement
    // double currentAngle = getEncoder() % (2 * Math.PI);
    // double setpoint = findClosestAngle(currentAngle, finalAngle);
    // double setpointFlipped = findClosestAngle(currentAngle, finalAngle +
    // Math.PI);

    // double angleDiff = Math.abs(currentAngle - finalAngle);
    // double adjustedVelocity = Math.cos(angleDiff) * velocityRPS;

    // if (Math.abs(setpoint) <= Math.abs(setpointFlipped)) {
    // motorTurn.setControl(positionTorqueFOCRequest.withPosition(currentAngle +
    // setpoint));
    // } else {
    // motorTurn.setControl(positionTorqueFOCRequest.withPosition(currentAngle +
    // setpointFlipped));
    // }
    // }
    // }

    private double MPSToRPS(double mps) {
        // convert meters per second to motor revolutions per second
        double wheelCirc = Constants.Physical.WHEEL_CIRCUMFERENCE;
        double wheelRPS = mps / wheelCirc;
        return wheelRPS * Constants.Ratios.DRIVE_GEAR_RATIO;
    }

    private double RPSToMPS(double rps) {

        double wheelCirc = Constants.Physical.WHEEL_CIRCUMFERENCE;
        double wheelRPS = rps / wheelCirc;
        return wheelRPS * Constants.Ratios.DRIVE_GEAR_RATIO;
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
