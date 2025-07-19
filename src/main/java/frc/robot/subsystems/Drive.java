package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.math.Vector;
import frc.robot.OI;
import frc.robot.tools.SwerveModule;
import frc.robot.tools.math.PID;

public class Drive extends SubsystemBase {
    private final Peripherals peripherals = new Peripherals();
    public final TalonFX driveMotor1, driveMotor2, driveMotor3, driveMotor4;
    public final TalonFX turnMotor1, turnMotor2, turnMotor3, turnMotor4;
    public final CANcoder encoder1, encoder2, encoder3, encoder4;
    private final SwerveModule swerve1, swerve2, swerve3, swerve4;
    private final SwerveDriveOdometry odometry;
    public static Pose2d m_pose = new Pose2d();
    private final PID xPID = new PID(0.01, 0.0, 0.0);
    private final PID yPID = new PID(0.01, 0.0, 0.0);

    public Drive() {
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
                new Translation2d(0.6096, 0.6096),
                new Translation2d(0.6096, -0.6096),
                new Translation2d(-0.6096, 0.6096),
                new Translation2d(-0.6096, -0.6096));
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

    public double getX() {
        return m_pose.getX() * -0.82;
    }

    public double getY() {
        return m_pose.getY() * -0.78;
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

    @Override
    public void periodic() {
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
        double rightX = OI.getDriverRightX() * 0.65;

        if (Math.abs(leftX) < 0.1) {
            leftX = 0;
        }
        if (Math.abs(leftY) < 0.1) {
            leftY = 0;
        }
        if (Math.abs(rightX) < 0.1) {
            rightX = 0;
        }

        Vector driveVector = new Vector(leftX, leftY);
        if (driveVector.magnitude() > 1.0) {
            driveVector = driveVector.scaled(1.0 / driveVector.magnitude());
        }

        double angleDeg = peripherals.getPigeonAngle();
        double angleRad = Math.toRadians(angleDeg);
        double cosA = Math.cos(angleRad);
        double sinA = Math.sin(angleRad);

        double fieldX = driveVector.getI() * cosA - driveVector.getJ() * sinA;
        double fieldY = driveVector.getI() * sinA + driveVector.getJ() * cosA;

        Vector fieldCentricVector = new Vector(fieldX, fieldY);

        driveAuto(fieldCentricVector, rightX);
    }
}
