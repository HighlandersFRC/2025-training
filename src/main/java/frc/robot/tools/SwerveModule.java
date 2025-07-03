package frc.robot.tools;

import frc.robot.Constants;
import frc.robot.tools.math.PID;
import frc.robot.tools.math.Vector;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private final TalonFX motorDrive;
    private final TalonFX motorTurn;
    private final CANcoder motorEncoder;
    private final Vector turnVector;
    private final double wheelCircumference = Constants.Physical.WHEEL_CIRCUMFERENCE;
    private final PID turnPID = new PID(0.01, 0.0, 0.0);

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder encoder, int index) {
        this.motorDrive = driveMotor;
        this.motorTurn = turnMotor;
        this.motorEncoder = encoder;
        this.turnVector = new Vector(0, 0);

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
    }

    public void drive(Vector driveVector, double turnInput) {
        double vx = driveVector.getI() + turnInput * turnVector.getI();
        double vy = driveVector.getJ() + turnInput * turnVector.getJ();

        double speed = Math.hypot(vx, vy);
        double desiredAngle = Math.toDegrees(Math.atan2(vy, vx));
        if (desiredAngle < 0)
            desiredAngle += 360;

        double currentAngle = getAngle();

        double delta = desiredAngle - currentAngle;
        if (delta > 180)
            delta -= 360;
        else if (delta < -180)
            delta += 360;

        if (Math.abs(delta) > 90) {
            desiredAngle = (desiredAngle + 180) % 360;
            speed = -speed;
        }

        setSpeed(speed);
        setDirection(desiredAngle);
    }

    private void setSpeed(double velocity) {
        motorDrive.set(velocity);
    }

    private void setDirection(double angle) {

        turnPID.setSetPoint(angle);
        double output = turnPID.updatePID(getAngle());
        motorTurn.set(-output);
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
