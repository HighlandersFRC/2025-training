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
    public boolean atPosition = false;

    public boolean startedPathToPoint = false;

    private PID xPID = new PID(0.5, 0, 1.2);
    private PID yPID = new PID(0.5, 0, 1.2);
    private PID yawPID = new PID(0.004, 0, 0.011);

    double targetX = 0;
    double targetY = 0;
    double targetAngle = 0;

    public enum DriveState {
        DEFAULT,
        PATH_TO_POINT,
        AUTO_PLACE,
        IDLE
    }

    private DriveState wantedState = DriveState.IDLE;
    private DriveState systemState = DriveState.IDLE;

    public Drive() {
        driveMotor1 = new TalonFX(1, "Canivore");
        driveMotor2 = new TalonFX(3, "Canivore");
        driveMotor3 = new TalonFX(5, "Canivore");
        driveMotor4 = new TalonFX(7, "Canivore");

        turnMotor1 = new TalonFX(2, "Canivore");
        turnMotor2 = new TalonFX(4, "Canivore");
        turnMotor3 = new TalonFX(6, "Canivore");
        turnMotor4 = new TalonFX(8, "Canivore");

        encoder1 = new CANcoder(1, "Canivore");
        encoder2 = new CANcoder(2, "Canivore");
        encoder3 = new CANcoder(3, "Canivore");
        encoder4 = new CANcoder(4, "Canivore");

        swerve1 = new SwerveModule(driveMotor1, turnMotor1, encoder1, 1);
        swerve2 = new SwerveModule(driveMotor2, turnMotor2, encoder2, 2);
        swerve3 = new SwerveModule(driveMotor3, turnMotor3, encoder3, 3);
        swerve4 = new SwerveModule(driveMotor4, turnMotor4, encoder4, 4);

        var kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.Swerve.moduleX,
                        Constants.Swerve.moduleY),
                new Translation2d(Constants.Swerve.moduleX, -Constants.Swerve.moduleY),
                new Translation2d(-Constants.Swerve.moduleX,
                        Constants.Swerve.moduleY),
                new Translation2d(-Constants.Swerve.moduleX, -Constants.Swerve.moduleY));
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

        yawPID.setMinInput(-180);
        yawPID.setMaxInput(180);
        yawPID.setContinuous(true);

        xPID.setMinOutput(-3.0);
        xPID.setMaxOutput(3.0);

        yPID.setMinOutput(-3.0);
        yPID.setMaxOutput(3.0);

        yawPID.setMinOutput(-1);
        yawPID.setMaxOutput(1);

    }

    public void setWantedState(DriveState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WANTED_GAME_PIECE {
        ALGAE,
        CORAL,
    }

    private DriveState handleStateTransition() {
        switch (wantedState) {
            case DEFAULT:
                if (systemState == DriveState.AUTO_PLACE) {
                    startedPathToPoint = false;
                    atPosition = false;
                }
                return DriveState.DEFAULT;
            case IDLE:
                return DriveState.IDLE;
            case PATH_TO_POINT:
                if (systemState == DriveState.DEFAULT) {
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
        return Math.toRadians(peripherals.getPigeonAngle());
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
    }

    public void driveSwerve(Vector driveVector, double turn) {
        swerve1.drive(driveVector, turn, Math.toDegrees(getAngle()));
        swerve2.drive(driveVector, turn, Math.toDegrees(getAngle()));
        swerve3.drive(driveVector, turn, Math.toDegrees(getAngle()));
        swerve4.drive(driveVector, turn, Math.toDegrees(getAngle()));
    }

    public void autoDrive(Vector fieldVector, double targetYawDegrees) {
        double vx = fieldVector.getI();
        double vy = -fieldVector.getJ();

        double headingRad = getAngle();
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double rx = vx * cosA - vy * sinA;
        double ry = vx * sinA + vy * cosA;

        Vector robotVector = new Vector(rx, ry);

        driveSwerve(robotVector, targetYawDegrees);
    }

    public void teleopDrive() {
        startedPathToPoint = false;

        double leftX = OI.getDriverLeftY();
        double leftY = -OI.getDriverLeftX();
        double rightX = Math.abs(OI.getDriverRightX()) < 0.03 ? 0 : OI.getDriverRightX() * 0.15;

        if (Math.abs(leftX) < 0.03)
            leftX = 0;
        if (Math.abs(leftY) < 0.03)
            leftY = 0;
        if (Math.abs(rightX) < 0.03)
            rightX = 0;

        double originalY = -(Math.copySign(leftY * leftY, leftY));
        double originalX = -(Math.copySign(leftX * leftX, leftX));

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

        double halfL = Constants.Swerve.chassisLengthMeters / 2.0;
        double halfW = Constants.Swerve.chassisWidthMeters / 2.0;
        double R = Math.sqrt(halfL * halfL + halfW * halfW);

        if (R <= 1e-6) {
            R = 1.0;
        }

        double scaledRotation = -rightX / R;

        if (scaledRotation > 1.0)
            scaledRotation = 1.0;
        if (scaledRotation < -1.0)
            scaledRotation = -1.0;

        driveSwerve(fieldCentricVector, scaledRotation);
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
        Logger.recordOutput("Robot X", getX());
        Logger.recordOutput("Robot Y", getY());
        Logger.recordOutput("Robot Angle", getAngle());
        Logger.recordOutput("Robot Pose", getPose2D());

    }

    public void goToPoint(double xOffset, double yOffset) {
        wantedState = DriveState.PATH_TO_POINT;
    }

    public boolean getAtPosition() {
        return atPosition;
    }

    public void moveToPoint(double targetX, double targetY, double targetAngle) {
        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        yawPID.setSetPoint(targetAngle);

        startedPathToPoint = true;

        double xOut = xPID.updatePID(getX()) / 1.0;
        double yOut = -yPID.updatePID(getY()) / 1.0;
        double turnOut = -yawPID.updatePID(Math.toDegrees(getAngle()));

        Logger.recordOutput("xPID Error", xPID.getError());
        Logger.recordOutput("yPID Error", yPID.getError());
        Logger.recordOutput("Yaw Error", yawPID.getError());

        boolean posClose = Math.abs(xPID.getError()) < 0.03
                && Math.abs(yPID.getError()) < 0.03;
        boolean angleClose = Math.abs(yawPID.getError()) < 1.0;

        Logger.recordOutput("YawPID Input", Math.toDegrees(getAngle()));
        Logger.recordOutput("Pos Close", posClose);
        Logger.recordOutput("Angle Close", angleClose);

        if (posClose && angleClose) {
            setWantedState(DriveState.DEFAULT);
        } else {
            atPosition = false;
            setWantedState(DriveState.AUTO_PLACE);
            autoDrive(new Vector(xOut, yOut), turnOut);
        }
    }

    public void moveToPoint() {
        xPID.setSetPoint(Constants.x);
        yPID.setSetPoint(Constants.y);
        yawPID.setSetPoint(Constants.angle);

        startedPathToPoint = true;

        double xOut = xPID.updatePID(getX()) / 1.0;
        double yOut = -yPID.updatePID(getY()) / 1.0;
        double turnOut = -yawPID.updatePID(Math.toDegrees(getAngle()));

        Logger.recordOutput("xPID Error", xPID.getError());
        Logger.recordOutput("yPID Error", yPID.getError());
        Logger.recordOutput("Yaw Error", yawPID.getError());

        boolean posClose = Math.abs(xPID.getError()) < 0.03
                && Math.abs(yPID.getError()) < 0.03;
        boolean angleClose = Math.abs(yawPID.getError()) < 1.0;

        Logger.recordOutput("YawPID Input", Math.toDegrees(getAngle()));
        Logger.recordOutput("Pos Close", posClose);
        Logger.recordOutput("Angle Close", angleClose);

        if (posClose && angleClose) {
        } else {
            atPosition = false;
            autoDrive(new Vector(xOut, yOut), turnOut);
        }
    }

    public double[] findClosestPiece(WANTED_GAME_PIECE piece, double x, double y) {
        java.util.List<Pose2d> a, b;

        if (piece == WANTED_GAME_PIECE.ALGAE) {
            if (isOnBlueSide()) {
                a = Constants.Reef.algaeBlueFrontPlacingPositions;
                b = Constants.Reef.algaeBlueBackPlacingPositions;
            } else {
                a = Constants.Reef.algaeRedFrontPlacingPositions;
                b = Constants.Reef.algaeRedBackPlacingPositions;
            }
        } else {
            if (isOnBlueSide()) {
                a = Constants.Reef.blueFrontPlacingPositions;
                b = Constants.Reef.blueBackPlacingPositions;
            } else {
                a = Constants.Reef.redFrontPlacingPositions;
                b = Constants.Reef.redBackPlacingPositions;
            }
        }

        double best = Double.POSITIVE_INFINITY;
        double rx = x, ry = y, rAngle = 0.0;

        if (a != null) {
            for (int i = 0; i < a.size(); i++) {
                Pose2d front = a.get(i);
                Pose2d back = (b != null && b.size() > i) ? b.get(i) : null;

                double distFront = Math.hypot(x - front.getX(), y - front.getY());
                if (distFront < best && distFront <= Constants.Autonomous.AUTO_PLACE_DISTANCE) {
                    best = distFront;
                    rx = front.getX();
                    ry = front.getY();
                    rAngle = front.getRotation().getRadians();
                }

                if (back != null) {
                    double distBack = Math.hypot(x - back.getX(), y - back.getY());
                    if (distBack < best && distBack <= Constants.Autonomous.AUTO_PLACE_DISTANCE) {
                        best = distBack;
                        rx = back.getX();
                        ry = back.getY();
                        rAngle = back.getRotation().getRadians() + Math.PI;
                    }
                }
            }
        }

        double[] out = new double[] { rx, ry, rAngle };
        try {
            Logger.recordOutput("closestPiece", out);
        } catch (Throwable t) {
            System.out.printf("closestPiece: (%.2f, %.2f, %.2f rad)%n", rx, ry, rAngle);
        }
        return out;
    }

    public boolean isOnBlueSide() {
        return false;
    }

    public Pose2d coordToPose2d(double x, double y) {
        return new Pose2d(x, y, new Rotation2d(0));
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
                moveToPoint(0, 0, 90);
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }
}