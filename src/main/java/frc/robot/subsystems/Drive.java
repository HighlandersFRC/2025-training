package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.math.Vector;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.SwerveModule;
import frc.robot.tools.math.PID;

public class Drive extends SubsystemBase {
    private final Peripherals peripherals = new Peripherals();
    public final TalonFX driveMotor1, driveMotor2, driveMotor3, driveMotor4;
    public final TalonFX turnMotor1, turnMotor2, turnMotor3, turnMotor4;
    Superstructure superstructure;
    public final CANcoder encoder1, encoder2, encoder3, encoder4;
    private final SwerveModule swerve1, swerve2, swerve3, swerve4;
    private final SwerveDriveOdometry odometry;
    public static Pose2d m_pose = new Pose2d();
    private final double offset = 0.6096 / 2;
    public boolean atPosition = false;

    private boolean startedPathToPoint = false;

    private PID xPID = new PID(1.5, 0, 3);
    private PID yPID = new PID(1.5, 0, 3);
    private PID yawPID = new PID(2, 0, 2.9);

    double targetX = 0;
    double targetY = 0;
    double targetAngle = 0;

    public enum DriveState {
        DEFAULT,
        PATH_TO_POINT,
        IDLE
    }

    private DriveState wantedState = DriveState.IDLE;
    private DriveState systemState = DriveState.IDLE;

    public Drive() {
        // xPID.setPID(0.9, 0, 0);
        // yPID.setPID(0.9, 0, 0);
        // yawPID.setPID(2.9, 0, 2);

        driveMotor1 = new TalonFX(1, "Canivore");
        driveMotor2 = new TalonFX(4, "Canivore");
        driveMotor3 = new TalonFX(6, "Canivore");
        driveMotor4 = new TalonFX(8, "Canivore");

        turnMotor1 = new TalonFX(2, "Canivore");
        turnMotor2 = new TalonFX(3, "Canivore");
        turnMotor3 = new TalonFX(5, "Canivore");
        turnMotor4 = new TalonFX(7, "Canivore");

        encoder1 = new CANcoder(1, "Canivore");
        encoder2 = new CANcoder(2, "Canivore");
        encoder3 = new CANcoder(3, "Canivore");
        encoder4 = new CANcoder(4, "Canivore");

        swerve1 = new SwerveModule(driveMotor1, turnMotor1, encoder1, 1);
        swerve2 = new SwerveModule(driveMotor2, turnMotor2, encoder2, 2);
        swerve3 = new SwerveModule(driveMotor3, turnMotor3, encoder3, 3);
        swerve4 = new SwerveModule(driveMotor4, turnMotor4, encoder4, 4);

        var kinematics = new SwerveDriveKinematics(
                new Translation2d(offset, offset),
                new Translation2d(offset, -offset),
                new Translation2d(-offset, offset),
                new Translation2d(-offset, -offset));
        odometry = new SwerveDriveOdometry(
                kinematics,
                peripherals.getRotation2d(),
                new SwerveModulePosition[] {
                        swerve2.getPosition(),
                        swerve1.getPosition(),
                        swerve3.getPosition(),
                        swerve4.getPosition()
                },
                new Pose2d());

    }

    public void setWantedState(DriveState wantedState) {
        this.wantedState = wantedState;
    }

    private DriveState handleStateTransition() {
        switch (wantedState) {
            case DEFAULT:
                if (systemState == DriveState.PATH_TO_POINT) {
                    startedPathToPoint = false;
                    atPosition = false;
                }
                return DriveState.DEFAULT;
            case IDLE:
                return DriveState.IDLE;
            case PATH_TO_POINT:
                if (systemState != DriveState.PATH_TO_POINT) {
                    startedPathToPoint = false;
                    atPosition = false;
                }

                if (systemState == DriveState.PATH_TO_POINT && startedPathToPoint) {
                    boolean posClose = Math.abs(xPID.getError()) < 0.03
                            && Math.abs(yPID.getError()) < 0.03;
                    boolean angleClose = Math.abs(yawPID.getError()) < 1.0;

                    if (posClose && angleClose) {
                        atPosition = true;
                        return DriveState.DEFAULT;
                    }
                }
                return DriveState.PATH_TO_POINT;
            default:
                return DriveState.IDLE;
        }
    }

    public double getX() {
        return m_pose.getX();
    }

    public double getY() {
        return m_pose.getY();
    }

    public double getAngle() {
        return m_pose.getRotation().getDegrees();
    }

    public Rotation2d getRotation2D() {
        return peripherals.getRotation2d();
    }

    public Pose2d getPose2D() {
        Pose2d pose = new Pose2d(getX(), getY(), getRotation2D());
        return pose;
    }

    public void stop() {
        swerve1.stop();
        swerve2.stop();
        swerve3.stop();
        swerve4.stop();
    }

    public void drive(double leftx, double lefty, double rightx, double yaw) {
        double leftX = lefty;
        double leftY = -leftX;
        double rightX = rightx;

        Vector driveVector = new Vector(leftX, leftY);
        if (driveVector.magnitude() > 1.0) {
            driveVector = driveVector.scaled(1.0 / driveVector.magnitude());
        }

        // swerve1.drive(driveVector, rightX, yaw);
        // swerve2.drive(driveVector, rightX, yaw);
        // swerve3.drive(driveVector, rightX, yaw);
        // swerve4.drive(driveVector, rightX, yaw);
    }

    public void driveAuto(Vector driveVector, double turn) {
        swerve1.drive(driveVector, turn, getAngle());
        swerve2.drive(driveVector, turn, getAngle());
        swerve3.drive(driveVector, turn, getAngle());
        swerve4.drive(driveVector, turn, getAngle());
    }

    public void autoDrive(Vector fieldVector, double targetYawDegrees) {
        double vx = fieldVector.getI();
        double vy = -fieldVector.getJ();

        double headingRad = Math.toRadians(m_pose.getRotation().getDegrees());
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double rx = vx * cosA - vy * sinA;
        double ry = vx * sinA + vy * cosA;

        Vector robotVector = new Vector(rx, ry);

        driveAuto(robotVector, targetYawDegrees);
    }

    public void teleopDrive() {
        if (OI.getDriverA()) {
            peripherals.zeroPigeon();
        }

        double leftX = OI.getDriverLeftY();
        double leftY = -OI.getDriverLeftX();
        double rightX = Math.abs(OI.getDriverRightX()) < 0.03 ? 0 : OI.getDriverRightX() * 0.15;

        double originalY = -(Math.copySign(leftY * leftY, leftY));
        double originalX = -(Math.copySign(leftX * leftX, leftX));

        if (Math.abs(leftX) < 0.03) {
            leftX = 0;
        }

        if (Math.abs(leftY) < 0.03) {
            leftY = 0;
        }

        if (Math.abs(rightX) < 0.03) {
            rightX = 0;
        }

        Vector driveVector = new Vector(originalX, originalY);
        if (driveVector.magnitude() > 1.0) {
            driveVector = driveVector.scaled(1.0 / driveVector.magnitude());
        }

        Vector fieldCentricVector;
        if (driveVector.magnitude() > 0) {
            double angleDeg = peripherals.getPigeonAngle();
            double angleRad = Math.toRadians(angleDeg);
            double cosA = Math.cos(angleRad);
            double sinA = Math.sin(angleRad);

            double fieldX = driveVector.getI() * cosA - driveVector.getJ() * sinA;
            double fieldY = driveVector.getI() * sinA + driveVector.getJ() * cosA;

            fieldCentricVector = new Vector(fieldX, fieldY);
        } else {
            fieldCentricVector = new Vector(0, 0);
        }

        driveAuto(fieldCentricVector, -rightX);
    }

    public void updateOdometry() {
        m_pose = odometry.update(
                peripherals.getRotation2d(),
                new SwerveModulePosition[] {
                        swerve2.getPosition(),
                        swerve1.getPosition(),
                        swerve3.getPosition(),
                        swerve4.getPosition()
                });
        Logger.recordOutput("Robot X", m_pose.getX());
        Logger.recordOutput("Robot Y", m_pose.getY());
        Logger.recordOutput("Robot Angle", m_pose.getRotation().getDegrees());
        Logger.recordOutput("Robot Pose", getPose2D());

    }

    public void goToPoint(double xOffset, double yOffset) {
        wantedState = DriveState.PATH_TO_POINT;
    }

    public boolean getAtPosition() {
        return atPosition;
    }

    @Override
    public void periodic() {
        updateOdometry();

        DriveState newState = handleStateTransition();

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEFAULT:
                teleopDrive();
                break;
            case PATH_TO_POINT:
                if (!startedPathToPoint) {

                    targetX = getX() + 0.3;
                    targetY = getY() + 0.3;
                    targetAngle = getAngle();

                    yawPID.setMinInput(-180);
                    yawPID.setMaxInput(180);
                    yawPID.setContinuous(true);

                    xPID.setMinOutput(-3.0);
                    xPID.setMaxOutput(3.0);

                    yPID.setMinOutput(-3.0);
                    yPID.setMaxOutput(3.0);

                    yawPID.setMinOutput(-0.5);
                    yawPID.setMaxOutput(0.5);

                    xPID.setSetPoint(targetX);
                    yPID.setSetPoint(targetY);
                    yawPID.setSetPoint(targetAngle);

                    startedPathToPoint = true;
                }

                double xOut = xPID.updatePID(getX()) / 2.0;
                double yOut = -yPID.updatePID(getY()) / 2.0;
                double turnOut = -yawPID.updatePID(getAngle()) / 8.0;

                Logger.recordOutput("xPID Error", xPID.getError());
                Logger.recordOutput("yPID Error", yPID.getError());
                Logger.recordOutput("Yaw Error", yawPID.getError());

                boolean posClose = Math.abs(xPID.getError()) < 0.03
                        && Math.abs(yPID.getError()) < 0.03;
                boolean angleClose = Math.abs(yawPID.getError()) < 1.0;

                Logger.recordOutput("Pos Close", posClose);
                Logger.recordOutput("Angle Close", angleClose);

                if (posClose && angleClose) {
                    System.out.println("Path to point complete - switching to DEFAULT state");
                } else {
                    atPosition = false;
                    autoDrive(new Vector(xOut, yOut), turnOut);
                }
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }
}