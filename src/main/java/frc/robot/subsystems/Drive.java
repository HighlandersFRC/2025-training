package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {
    private final TalonFX frontRightDriveMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_DRIVE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX frontRightAngleMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_ANGLE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX frontLeftDriveMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_DRIVE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX frontLeftAngleMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_ANGLE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX backLeftDriveMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_DRIVE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX backLeftAngleMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_ANGLE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX backRightDriveMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_DRIVE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX backRightAngleMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_ANGLE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);

    private final CANcoder frontRightCanCoder = new CANcoder(Constants.CANInfo.FRONT_RIGHT_MODULE_CANCODER_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANcoder frontLeftCanCoder = new CANcoder(Constants.CANInfo.FRONT_LEFT_MODULE_CANCODER_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANcoder backLeftCanCoder = new CANcoder(Constants.CANInfo.BACK_LEFT_MODULE_CANCODER_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANcoder backRightCanCoder = new CANcoder(Constants.CANInfo.BACK_RIGHT_MODULE_CANCODER_ID,
            Constants.CANInfo.CANBUS_NAME);

    public boolean getSwerveCAN() { // checks to see if all of the swerve motors and encoders are connected
        if (getSwerveMotorsConnected() == 8 && getSwerveCANCodersConnected() == 4) {
            return true;
        } else
            return false;
    }

    public int getSwerveMotorsConnected() { // counts all of the swerve motors that are connected
        int count = 0;
        if (frontRightDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (frontLeftDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (backRightDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (backLeftDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (frontRightAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (frontLeftAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (backRightAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        if (backLeftAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
            count++;
        }
        return count;
    }

    public int getSwerveCANCodersConnected() { // counts all of the swerve encoders that are connected
        int count = 0;
        if (frontRightCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
            count++;
        }
        if (frontLeftCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
            count++;
        }
        if (backRightCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
            count++;
        }
        if (backLeftCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
            count++;
        }
        return count;
    }

    // creates all 4 modules
    private final SwerveModule frontRight = new SwerveModule(1, frontRightAngleMotor, frontRightDriveMotor,
            frontRightCanCoder);
    private final SwerveModule frontLeft = new SwerveModule(2, frontLeftAngleMotor, frontLeftDriveMotor,
            frontLeftCanCoder);
    private final SwerveModule backLeft = new SwerveModule(3, backLeftAngleMotor, backLeftDriveMotor, backLeftCanCoder);
    private final SwerveModule backRight = new SwerveModule(4, backRightAngleMotor, backRightDriveMotor,
            backRightCanCoder);

    Peripherals peripherals;
    boolean firstClimb = false;

    // xy position of module based on robot width and distance from edge of robot
    private final double moduleX = ((Constants.Physical.ROBOT_LENGTH) / 2) - Constants.Physical.MODULE_OFFSET;
    private final double moduleY = ((Constants.Physical.ROBOT_WIDTH) / 2) - Constants.Physical.MODULE_OFFSET;

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
    Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
    Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
    Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

    // odometry
    private double m_currentX = 0;
    private double m_currentY = 0;
    private double m_currentTheta = 0;

    private double m_averagedX = 0.0;
    private double m_averagedY = 0.0;
    private double m_averagedTheta = 0.0;

    private double m_initTime;
    private double m_currentTime;

    // array for fused odometry
    private double[] currentFusedOdometry = new double[3];

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    SwerveDrivePoseEstimator m_odometry;
    Pose2d m_pose;

    SwerveDrivePoseEstimator loggingOdometry;
    Pose2d loggingPose;

    SwerveDrivePoseEstimator mt2Odometry;
    Pose2d mt2Pose;

    PhotonPoseEstimator photonPoseEstimator;
    PhotonPoseEstimator backPhotonPoseEstimator;
    PhotonPoseEstimator backLeftPhotonPoseEstimator;
    PhotonPoseEstimator backRightPhotonPoseEstimator;

    // PhotonPoseEstimator rightPhotonPoseEstimator;
    // PhotonPoseEstimator leftPhotonPoseEstimator;
    PhotonPoseEstimator swervePhotonPoseEstimator;
    PhotonPoseEstimator gamePiecePhotonPoseEstimator;
    AprilTagFieldLayout aprilTagFieldLayout;

    // *********************NOTE THE PITCH IS POSITIVE DOWNWARDS
    // **********************************

    Transform3d frontReefRobotToCam = new Transform3d( // top front reef cam
            new Translation3d(Constants.inchesToMeters(2.0), Constants.inchesToMeters(-11.5),
                    Constants.inchesToMeters(23.625)),
            new Rotation3d(Math.toRadians(0.3), Math.toRadians(25.6), Math.toRadians(15.0)));

    Transform3d frontSwerveRobotToCam = new Transform3d( // front reef cam on swerve module
            new Translation3d(Constants.inchesToMeters(11.75),
                    Constants.inchesToMeters(-8.5),
                    Constants.inchesToMeters(8.75)),
            new Rotation3d(Math.toRadians(1.1), Math.toRadians(15.3),
                    Math.toRadians(35.0)));

    Transform3d backReefRobotToCam = new Transform3d( // top back reef cam
            new Translation3d(Constants.inchesToMeters(-2.0), Constants.inchesToMeters(-11.5),
                    Constants.inchesToMeters(23.625)),
            new Rotation3d(Math.toRadians(1.5), Math.toRadians(25.2), Math.toRadians(165.0)));

    Transform3d backLeftReefRobotToCam = new Transform3d(
            new Translation3d(Constants.inchesToMeters(-12.375), Constants.inchesToMeters(9.375),
                    Constants.inchesToMeters(8.6875)),
            new Rotation3d(Math.toRadians(0.5), Math.toRadians(-20.7), Math.toRadians(181.53))); // 0.4, -20.5

    Transform3d backRightReefRobotToCam = new Transform3d(
            new Translation3d(Constants.inchesToMeters(
                    -12.375), Constants.inchesToMeters(-9.25),
                    Constants.inchesToMeters(8.6875)),
            new Rotation3d(Math.toRadians(0.5), Math.toRadians(-22.5), Math.toRadians(178.47)));

    Transform3d gamePieceReefRobotToCam = new Transform3d(
            new Translation3d(Constants.inchesToMeters(2.0), Constants.inchesToMeters(-11.5),
                    Constants.inchesToMeters(20.25)),
            new Rotation3d(Math.toRadians(1.0), Math.toRadians(21.4), Math.toRadians(15.0)));

    double initAngle;
    double setAngle;
    double diffAngle;

    // path following PID values
    private double kXP = 4.00;
    private double kXI = 0.00;
    private double kXD = 1.20;

    private double kYP = kXP;
    private double kYI = kXI;
    private double kYD = kXD;

    private double kThetaP = 2.90;
    private double kThetaI = 0.00;
    private double kThetaD = 2.00;

    // auto placement PID values
    private double kkXP = 5.50;
    private double kkXI = 0.00;
    private double kkXD = 1.60;

    private double kkYP = kkXP;
    private double kkYI = kkXI;
    private double kkYD = kkXD;

    private double kkThetaP = 1.79;
    private double kkThetaI = 0.00;
    private double kkThetaD = 0.971;

    // l4 pid values
    private double kkXP4 = 4.30;
    private double kkXI4 = 0.00;
    private double kkXD4 = 1.70;

    private double kkYP4 = kkXP4;
    private double kkYI4 = kkXI4;
    private double kkYD4 = kkXD4;

    private double kkThetaP4 = 2.056;
    private double kkThetaI4 = 0.00;
    private double kkThetaD4 = 0.971;

    // l4 auto pids
    private double scalar = 0.9;
    private double kkXP4A = 4.30 * scalar;
    private double kkXI4A = 0.00 * scalar;
    private double kkXD4A = 1.70 * scalar;

    private double kkYP4A = kkXP4A;
    private double kkYI4A = kkXI4A;
    private double kkYD4A = kkXD4A;

    private double kkThetaP4A = 2.056;
    private double kkThetaI4A = 0.00;
    private double kkThetaD4A = 0.971;

    // l23 pid values
    private double kkXP23 = 4.00;
    private double kkXI23 = 0.00;
    private double kkXD23 = 1.60;

    private double kkYP23 = kkXP23;
    private double kkYI23 = kkXI23;
    private double kkYD23 = kkXD23;

    private double kkThetaP23 = 2.481;
    private double kkThetaI23 = 0.00;
    private double kkThetaD23 = 0.971;

    // l1 pid values
    private double kkXP1 = 4.00;
    private double kkXI1 = 0.00;
    private double kkXD1 = 1.40;

    private double kkYP1 = kkXP1;
    private double kkYI1 = kkXI1;
    private double kkYD1 = kkXD1;

    private double kkThetaP1 = 3.005;
    private double kkThetaI1 = 0.00;
    private double kkThetaD1 = 0.971;

    // Piece pickup values
    private double kkkXPPickup = 3.30;
    private double kkkXIPickup = 0.00;
    private double kkkXDPickup = 1.70;

    private double kkkYPPickup = kkkXPPickup;
    private double kkkYIPickup = kkkXIPickup;
    private double kkkYDPickup = kkkXDPickup;

    private double kkkThetaPPickup = 2.056;
    private double kkkThetaIPickup = 0.00;
    private double kkkThetaDPickup = 0.971;

    // teleop targeting PID values
    private double kTurningP = 0.04;
    private double kTurningI = 0;
    private double kTurningD = 0.06;
    private double kRotateP = 0.04;
    private double kRotateI = 0.0;
    private double kRotateD = 0.06;

    private PID xxPID = new PID(kkXP, kkXI, kkXD);
    private PID yyPID = new PID(kkYP, kkYI, kkYD);
    private PID thetaaPID = new PID(kkThetaP, kkThetaI, kkThetaD);

    private PID xxPID4 = new PID(kkXP4, kkXI4, kkXD4);
    private PID yyPID4 = new PID(kkYP4, kkYI4, kkYD4);
    private PID thetaaPID4 = new PID(kkThetaP4, kkThetaI4, kkThetaD4);

    private PID xxPID4A = new PID(kkXP4A, kkXI4A, kkXD4A);
    private PID yyPID4A = new PID(kkYP4A, kkYI4A, kkYD4A);
    private PID thetaaPID4A = new PID(kkThetaP4A, kkThetaI4A, kkThetaD4A);

    private PID xxPID23 = new PID(kkXP23, kkXI23, kkXD23);
    private PID yyPID23 = new PID(kkYP23, kkYI23, kkYD23);
    private PID thetaaPID23 = new PID(kkThetaP23, kkThetaI23, kkThetaD23);

    private PID xxPID1 = new PID(kkXP1, kkXI1, kkXD1);
    private PID yyPID1 = new PID(kkYP1, kkYI1, kkYD1);
    private PID thetaaPID1 = new PID(kkThetaP1, kkThetaI1, kkThetaD1);

    private PID xxPIDPickup = new PID(kkkXPPickup, kkkXIPickup, kkkXDPickup);
    private PID yyPIDPickup = new PID(kkkYPPickup, kkkYIPickup, kkkYDPickup);
    private PID thetaaPIDPickup = new PID(kkkThetaPPickup, kkkThetaIPickup, kkkThetaDPickup);

    private PID xPID = new PID(kXP, kXI, kXD);
    private PID yPID = new PID(kYP, kYI, kYD);
    private PID thetaPID = new PID(kThetaP, kThetaI, kThetaD);
    private PID turningPID = new PID(kTurningP, kTurningI, kTurningD);
    private PID rotatePID = new PID(kRotateP, kRotateI, kRotateD);

    private String m_fieldSide = "blue";

    private double angleSetpoint = 0;
    double startX;
    double startY;

    private Pose2d targetPointPickup = new Pose2d();

    public boolean algaeMode = false;

    public enum DriveState {
        DEFAULT,
        IDLE,
        STOP,
        REEF,
        REEF_MORE,
        BACK,
        L3_REEF,
        L4_REEF,
        ALGAE,
        ALGAE_MORE,
        ALGAE_MORE_MORE,
        PROCESSOR,
        PROCESSOR_MORE,
        NET,
        NET_MORE,
        FEEDER,
        SCORE_L23,
        FEEDER_ALIGN,
        AUTO_FEEDER,
        AUTO_L1,
        AUTO_L1_MORE,
        FEEDER_AUTO,
        PIECE_PICKUP,
        AUTO_CLIMB,
        MOVE_TO_POINT
    }

    private DriveState wantedState = DriveState.IDLE;
    private DriveState systemState = DriveState.IDLE;

    /**
     * Creates a new instance of the Swerve Drive subsystem.
     * Initializes the Swerve Drive subsystem with the provided peripherals.
     * 
     * @param peripherals The peripherals used by the Swerve Drive subsystem.
     */
    Elevator elevator;

    public Drive(Peripherals peripherals, Elevator elevator) {
        this.peripherals = peripherals;
        this.elevator = elevator;

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getCanCoderPositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontRight.getCanCoderPositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backLeft.getCanCoderPositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backRight.getCanCoderPositionRadians()));

        Pose2d m_pose = new Pose2d();

        m_odometry = new SwerveDrivePoseEstimator(m_kinematics,
                new Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))), swerveModulePositions, m_pose);
        loggingOdometry = new SwerveDrivePoseEstimator(m_kinematics,
                new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions, m_pose);
        mt2Odometry = new SwerveDrivePoseEstimator(m_kinematics,
                new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions, m_pose);
    }

    // public boolean atSetpoint() {
    // double currentAngle = peripherals.getPigeonAngle();
    // if (getFieldSide().equals("red")) {
    // currentAngle -= 180;
    // }
    // return (Math.abs(Constants.standardizeAngleDegrees(currentAngle)
    // - Constants.standardizeAngleDegrees(angleSetpoint)) < 2);
    // }

    public void setWantedState(DriveState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(DriveState wantedState, double angle) {
        this.wantedState = wantedState;
        setSetpointAngle(angle);
    }

    public void setSetpointAngle(double angle) {
        this.angleSetpoint = angle;
        // Logger.recordOutput("Setpoint Theta: ", angle);
    }

    /**
     * Initializes the robot with the specified field side configuration.
     * It sets up configurations when run on robot initialization, such as setting
     * the field side,
     * initializing each swerve module, configuring motor inversions, and setting
     * PID controller output limits.
     * Additionally, it sets the default command to the DriveDefault command.
     *
     * @param fieldSide The side of the field (e.g., "red" or "blue").
     */
    public void init(String fieldSide) {
        // sets configurations when run on robot initalization
        this.m_fieldSide = fieldSide;

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().getPath() + "/" + "2025-reefscape-welded.json");
        } catch (Exception e) {
            java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontReefRobotToCam);

        backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backReefRobotToCam);

        backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftReefRobotToCam);

        backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightReefRobotToCam);

        swervePhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontSwerveRobotToCam);

        // rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontBargeRobotToCam);

        // leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backBargeRobotToCam);

        gamePiecePhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, gamePieceReefRobotToCam);

        frontRight.init();
        frontLeft.init();
        backRight.init();
        backLeft.init();

        xxPID.setMinOutput(-3.0);
        xxPID.setMaxOutput(3.0);

        yyPID.setMinOutput(-3.0);
        yyPID.setMaxOutput(3.0);

        thetaaPID.setMinOutput(-3.0);
        thetaaPID.setMaxOutput(3.0);

        xxPID4.setMinOutput(-2.0);
        xxPID4.setMaxOutput(2.0);

        yyPID4.setMinOutput(-2.0);
        yyPID4.setMaxOutput(2.0);

        thetaaPID4.setMinOutput(-2.0);
        thetaaPID4.setMaxOutput(2.0);

        xxPID4A.setMinOutput(-1.4);
        xxPID4A.setMaxOutput(1.4);

        yyPID4A.setMinOutput(-1.4);
        yyPID4A.setMaxOutput(1.4);

        thetaaPID4A.setMinOutput(-1.4);
        thetaaPID4A.setMaxOutput(1.4);

        xxPID23.setMinOutput(-4.0);
        xxPID23.setMaxOutput(4.0);

        yyPID23.setMinOutput(-4.0);
        yyPID23.setMaxOutput(4.0);

        thetaaPID23.setMinOutput(-4.0);
        thetaaPID23.setMaxOutput(4.0);

        xxPID1.setMinOutput(-4.0);
        xxPID1.setMaxOutput(4.0);

        yyPID1.setMinOutput(-4.0);
        yyPID1.setMaxOutput(4.0);

        thetaaPID1.setMinOutput(-4.0);
        thetaaPID1.setMaxOutput(4.0);

        xxPIDPickup.setMinOutput(-1.0);
        xxPIDPickup.setMaxOutput(1.0);

        yyPIDPickup.setMinOutput(-1.0);
        yyPIDPickup.setMaxOutput(1.0);

        thetaaPIDPickup.setMinOutput(-2.0);
        thetaaPIDPickup.setMaxOutput(2.0);

        xPID.setMinOutput(-4.9);
        xPID.setMaxOutput(4.9);

        yPID.setMinOutput(-4.9);
        yPID.setMaxOutput(4.9);

        thetaPID.setMinOutput(-3);
        thetaPID.setMaxOutput(3);

        turningPID.setMinOutput(-3);
        turningPID.setMaxOutput(3);

        rotatePID.setMinOutput(-2);
        rotatePID.setMaxOutput(2);
    }

    public void teleopInit() {
        angleSetpoint = peripherals.getPigeonAngle();
        if (getFieldSide() == "red") {
            angleSetpoint -= 180;
        }
        turningPID.setSetPoint(angleSetpoint);
        if (Math.abs(turningPID.getSetPoint() - angleSetpoint) > 100) {
            turningPID.setSetPoint(angleSetpoint);
        }

        frontLeft.setDriveCurrentLimits(120, 120); // if robot starts browning out in a placement sequence, just drop
                                                   // these
                                                   // to 40
        frontRight.setDriveCurrentLimits(120, 120);
        backLeft.setDriveCurrentLimits(120, 120);
        backRight.setDriveCurrentLimits(120, 120);
    }

    public void teleopPeriodic() {
        if (Math.abs(turningPID.getSetPoint() - angleSetpoint) > 100) {
            turningPID.setSetPoint(angleSetpoint);
        }
    }

    /**
     * Zeros the IMU (Inertial Measurement Unit) mid-match and resets the odometry
     * with a zeroed angle.
     * It resets the angle reported by the pigeon sensor to zero and updates the
     * odometry with this new zeroed angle.
     */
    public void zeroIMU() {
        angleSetpoint = 0;
        turningPID.setSetPoint(angleSetpoint);
        peripherals.zeroPigeon();
        // SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        // swerveModulePositions[0] = new
        // SwerveModulePosition(frontLeft.getModuleDistance(),
        // new Rotation2d(frontLeft.getCanCoderPositionRadians()));
        // swerveModulePositions[1] = new
        // SwerveModulePosition(frontRight.getModuleDistance(),
        // new Rotation2d(frontRight.getCanCoderPositionRadians()));
        // swerveModulePositions[2] = new
        // SwerveModulePosition(backLeft.getModuleDistance(),
        // new Rotation2d(backLeft.getCanCoderPositionRadians()));
        // swerveModulePositions[3] = new
        // SwerveModulePosition(backRight.getModuleDistance(),
        // new Rotation2d(backRight.getCanCoderPositionRadians()));

        // m_pose = m_odometry.update(new
        // Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))),
        // swerveModulePositions);
        // loggingPose = loggingOdometry.update(new Rotation2d(
        // Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions);
        // mt2Pose = mt2Odometry.update(new
        // Rotation2d(Math.toRadians(peripherals.getPigeonAngle())),
        // swerveModulePositions);
    }

    /**
     * Adjusts the angle reported by the pigeon sensor after an autonomous routine.
     * It adds 180 degrees to the current angle reported by the pigeon sensor and
     * wraps it around 360 degrees.
     */
    public void setPigeonAfterAuto() {
        peripherals.setPigeonAngle((peripherals.getPigeonAngle() + 180) % 360);
    }

    /**
     * Sets the angle reported by the pigeon sensor to the specified value.
     *
     * @param angle The angle to set for the pigeon sensor in degrees.
     */
    public void setPigeonAngle(double angle) {
        peripherals.setPigeonAngle(angle);
    }

    /**
     * Retrieves the current angle reported by the pigeon sensor.
     *
     * @return The current angle reported by the pigeon sensor in degrees.
     */
    public double getPigeonAngle() {
        return peripherals.getPigeonAngle();
    }

    /**
     * Sets the PID values for all swerve modules to zero, keeping the wheels
     * straight.
     */
    public void setWheelsStraight() {
        frontRight.setWheelPID(0.0, 0.0);
        frontLeft.setWheelPID(0.0, 0.0);
        backLeft.setWheelPID(0.0, 0.0);
        backRight.setWheelPID(0.0, 0.0);
    }

    /**
     * Initializes the robot's state for autonomous mode based on the provided path
     * points.
     * 
     * @param pathPoints The array of path points representing the trajectory for
     *                   the autonomous routine.
     */
    public void autoInit(JSONArray pathPoints) {
        // runs at start of autonomous
        java.util.logging.Logger.getGlobal().info("Auto init");
        JSONObject firstPoint = pathPoints.getJSONObject(0);
        double firstPointX = firstPoint.getDouble("x");
        double firstPointY = firstPoint.getDouble("y");
        double firstPointAngle = firstPoint.getDouble("angle");

        // changing odometry if on red side, don't need to change y because it will be
        // the same for autos on either side
        if (this.m_fieldSide == "blue") {
            firstPointX = Constants.Physical.FIELD_LENGTH - firstPointX;
            firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
            firstPointAngle = Math.PI + firstPointAngle;
        }

        if (OI.isProcessorSide()) {
            firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
            firstPointAngle = -firstPointAngle;
        }

        frontLeft.setDriveCurrentLimits(60, 120);
        frontRight.setDriveCurrentLimits(60, 120);
        backLeft.setDriveCurrentLimits(60, 120);
        backRight.setDriveCurrentLimits(60, 120);

        peripherals.setPigeonAngle(Math.toDegrees(firstPointAngle));
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
                new Rotation2d(frontRight.getCanCoderPositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
                new Rotation2d(backLeft.getCanCoderPositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
                new Rotation2d(backRight.getCanCoderPositionRadians()));
        m_odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
                new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
        loggingOdometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
                new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
        mt2Odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
                new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));

        currentFusedOdometry[0] = firstPointX;
        currentFusedOdometry[1] = firstPointY;
        currentFusedOdometry[2] = firstPointAngle;

        m_currentX = currentFusedOdometry[0];
        m_currentY = currentFusedOdometry[1];
        m_currentTheta = currentFusedOdometry[2];

        m_initTime = Timer.getFPGATimestamp();

        updateOdometryFusedArray();
    }

    /**
     * Sets the current field side designation.
     * 
     * @param side The field side designation to set, indicating whether the robot
     *             is positioned on the "blue" or "red" side of the field.
     */
    public void setFieldSide(String side) {
        m_fieldSide = side;
    }

    /**
     * Retrieves the current field side designation.
     * 
     * @return The current field side designation, indicating whether the robot is
     *         positioned on the "blue" or "red" side of the field.
     */
    public String getFieldSide() {
        return m_fieldSide;
    }

    /**
     * Retrieves the current timestamp relative to the start of the robot operation.
     * 
     * @return The current timestamp in seconds since the start of the robot
     *         operation.
     */
    public double getCurrentTime() {
        return m_currentTime;
    }

    public void addVisionMeasurementToOdometry(Pose2d visionPose, double timestamp) {
        m_odometry.addVisionMeasurement(
                visionPose,
                timestamp);
    }

    public void setOdometry(Pose2d pose) {
        // Logger.recordOutput("Odometry Reset to:", pose.toString());
        java.util.logging.Logger.getGlobal().fine("New Odometry Pose: " + pose.toString());
        m_odometry.resetPose(pose);
        loggingOdometry.resetPose(pose);
        mt2Odometry.resetPose(pose);
    }

    public boolean isPoseInField(Pose2d pose) {
        if (pose.getY() < 0 || pose.getY() > Constants.Physical.FIELD_WIDTH || pose.getX() < 0
                || pose.getX() > Constants.Physical.FIELD_LENGTH) {
            return false;
        } else {
            return true;
        }
    }

    private int getClosestTagId(Pose2d pose) {
        int closestTag = 0;
        double closestDistance = Double.MAX_VALUE;

        for (int i = 1; i <= aprilTagFieldLayout.getTags().size(); i++) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(i);
            if (tagPose.isPresent()) {
                double distance = Constants.Vision.distBetweenPose2d(pose, tagPose.get().toPose2d());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTag = i;
                }
            }
        }
        return closestTag;
    }

    private boolean inReefInteractionState() {
        return systemState == DriveState.L4_REEF || systemState == DriveState.L3_REEF ||
                systemState == DriveState.REEF || systemState == DriveState.AUTO_L1 ||
                systemState == DriveState.AUTO_L1_MORE || systemState == DriveState.ALGAE ||
                systemState == DriveState.ALGAE_MORE || systemState == DriveState.ALGAE_MORE_MORE;
    }

    private boolean closeToReef() {
        if (distanceFromCenterOfReef() < 2.7) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Updates the fused odometry array with current robot position and orientation
     * information.
     * Calculates the robot's position and orientation using swerve module positions
     * and the gyro angle.
     * Updates the current X, Y, and theta values, as well as previous values and
     * time differences.
     */
    public void updateOdometryFusedArray() {
        double navxOffset = Math.toRadians(peripherals.getPigeonAngle());

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
                new Rotation2d(frontRight.getCanCoderPositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
                new Rotation2d(backLeft.getCanCoderPositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
                new Rotation2d(backRight.getCanCoderPositionRadians()));

        m_pose = m_odometry.update(new Rotation2d((navxOffset)), swerveModulePositions);
        loggingPose = loggingOdometry.update(new Rotation2d(navxOffset), swerveModulePositions);
        mt2Pose = mt2Odometry.update(new Rotation2d(navxOffset), swerveModulePositions);

        m_currentX = getOdometryX();
        m_currentY = getOdometryY();
        m_currentTheta = navxOffset;

        Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
        Logger.recordOutput("Closde to reef", closeToReef());

        if (((!algaeMode && DriverStation.isTeleop()) || (DriverStation.isAutonomousEnabled() && closeToReef()))
                || (inReefInteractionState())) {
            photonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            backPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            backLeftPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            backRightPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            swervePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            gamePiecePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

            Rotation2d robotRotation = new Rotation2d(navxOffset);
            double time = Timer.getFPGATimestamp();
            photonPoseEstimator.addHeadingData(time, robotRotation);
            backPhotonPoseEstimator.addHeadingData(time, robotRotation);
            backLeftPhotonPoseEstimator.addHeadingData(time, robotRotation);
            backRightPhotonPoseEstimator.addHeadingData(time, robotRotation);
            swervePhotonPoseEstimator.addHeadingData(time, robotRotation);
            gamePiecePhotonPoseEstimator.addHeadingData(time, robotRotation);
        } else {
            photonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            backPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            backLeftPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            backRightPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            swervePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            gamePiecePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        }
        Logger.recordOutput("Back Strategy: ",
                backPhotonPoseEstimator.getPrimaryStrategy().toString());
        Logger.recordOutput("Back left strat: ", backLeftPhotonPoseEstimator.getPrimaryStrategy().toString());
        Logger.recordOutput("Back right strat: ",
                backRightPhotonPoseEstimator.getPrimaryStrategy().toString());
        // var result = peripherals.getFrontReefCamResult();
        // Optional<EstimatedRobotPose> multiTagResult =
        // photonPoseEstimator.update(result);
        // if (multiTagResult.isPresent()) {
        // if (result.getBestTarget().getPoseAmbiguity() < 0.3 &&
        // result.getBestTarget().fiducialId != 5
        // && result.getBestTarget().fiducialId != 4 &&
        // result.getBestTarget().fiducialId != 14
        // && result.getBestTarget().fiducialId != 15 &&
        // result.getBestTarget().fiducialId != 3
        // && result.getBestTarget().fiducialId != 16) {
        // Pose3d robotPose = multiTagResult.get().estimatedPose;
        // Logger.recordOutput("multitag result", robotPose);
        // int numFrontTracks = result.getTargets().size();
        // Pose3d tagPose =
        // aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get();
        // double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
        // // Logger.recordOutput("Distance to tag", distToTag);
        // if (distToTag < 3.2) {
        // if (systemState.equals(DriveState.REEF) ||
        // systemState.equals(DriveState.L3_REEF)
        // || systemState.equals(DriveState.L4_REEF)) {
        // standardDeviation.set(0, 0,
        // 0.5
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(1, 0,
        // 0.5
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(2, 0, 0.9);
        // } else {
        // standardDeviation.set(0, 0,
        // Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(1, 0,
        // Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(2, 0, 0.9);
        // }
        // // Pose2d poseWithoutAngle = new
        // Pose2d(robotPose.toPose2d().getTranslation(),
        // // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
        // mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
        // result.getTimestampSeconds());
        // }
        // }
        // }

        // // if (DriverStation.isTeleopEnabled()) {
        // // ArrayList<Pose2d> results = new ArrayList<Pose2d>();
        // // var gamePieceResult = peripherals.getFrontGamePieceCamResult();
        // // Optional<EstimatedRobotPose> gamePieceMultiTagResult =
        // // gamePiecePhotonPoseEstimator.update(gamePieceResult);
        // // if (gamePieceMultiTagResult.isPresent()) {
        // // if (gamePieceResult.getBestTarget().getPoseAmbiguity() < 0.3 &&
        // // gamePieceResult.getBestTarget().fiducialId != 5
        // // && gamePieceResult.getBestTarget().fiducialId != 4 &&
        // // gamePieceResult.getBestTarget().fiducialId != 14
        // // && gamePieceResult.getBestTarget().fiducialId != 15 &&
        // // gamePieceResult.getBestTarget().fiducialId != 3
        // // && gamePieceResult.getBestTarget().fiducialId != 16) {
        // // Pose3d robotPose = gamePieceMultiTagResult.get().estimatedPose;
        // // Logger.recordOutput("multitag result", robotPose);
        // // int numFrontTracks = gamePieceResult.getTargets().size();
        // // Pose3d tagPose =
        // //
        // aprilTagFieldLayout.getTagPose(gamePieceResult.getBestTarget().getFiducialId()).get();
        // // double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
        // // // Logger.recordOutput("Distance to tag", distToTag);
        // // if (distToTag < 3.2) {
        // if (systemState.equals(DriveState.REEF) ||
        // systemState.equals(DriveState.L3_REEF)
        // || systemState.equals(DriveState.L4_REEF)) {
        // standardDeviation.set(0, 0,
        // 0.5
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(1, 0,
        // 0.5
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(2, 0, 0.9);
        // } else {
        // standardDeviation.set(0, 0,
        // Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(1, 0,
        // Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
        // * Constants.Vision.getTagDistStdDevScalar(distToTag));
        // // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
        // // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
        // standardDeviation.set(2, 0, 0.9);
        // }
        // // Pose2d poseWithoutAngle = new
        // Pose2d(robotPose.toPose2d().getTranslation(),
        // // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
        // // mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
        // // gamePieceResult.getTimestampSeconds());
        // results.add(robotPose.toPose2d());
        // }
        // }
        // }

        if (getRobotSpeed() < 2.4) {
            var backResult = peripherals.getBackReefCamResult();
            Optional<EstimatedRobotPose> backMultiTagResult = backPhotonPoseEstimator.update(backResult);
            if (backMultiTagResult.isPresent()) {
                if (backResult.getBestTarget().getPoseAmbiguity() < 0.3 && backResult.getBestTarget().fiducialId != 5
                        && backResult.getBestTarget().fiducialId != 4 && backResult.getBestTarget().fiducialId != 14
                        && backResult.getBestTarget().fiducialId != 15 && backResult.getBestTarget().fiducialId != 3
                        && backResult.getBestTarget().fiducialId != 16) {
                    Pose3d robotPose = backMultiTagResult.get().estimatedPose;
                    Logger.recordOutput("multitag result", robotPose);
                    int numFrontTracks = backResult.getTargets().size();
                    Pose3d tagPose = aprilTagFieldLayout.getTagPose(backResult.getBestTarget().getFiducialId()).get();
                    double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                    // Logger.recordOutput("Distance to tag", distToTag);
                    if (distToTag < 3.2) {
                        if (inReefInteractionState()) {
                            standardDeviation.set(0, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            if (backResult.getBestTarget().getFiducialId() == getClosestTagId(getMt2Pose2d())) {
                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                        backResult.getTimestampSeconds());
                            }
                        } else {
                            standardDeviation.set(0, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                    backResult.getTimestampSeconds());
                        }
                        // Pose2d poseWithoutAngle = new Pose2d(robotPose.toPose2d().getTranslation(),
                        // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
                    }
                }
            }

            var swerveResult = peripherals.getFrontSwerveCamResult();
            Optional<EstimatedRobotPose> swerveMultiTagResult = swervePhotonPoseEstimator.update(swerveResult);
            if (swerveMultiTagResult.isPresent()
                    && (!inReefInteractionState()
                            || getAutoPlacementSideIsFront())) {
                if (swerveResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                    Pose3d robotPose = swerveMultiTagResult.get().estimatedPose;
                    int numFrontTracks = swerveResult.getTargets().size();
                    Pose3d tagPose = aprilTagFieldLayout.getTagPose(swerveResult.getBestTarget().getFiducialId()).get();
                    double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                    // Logger.recordOutput("Distance to tag", distToTag);
                    if (distToTag < 3.2) {
                        if (inReefInteractionState()) {
                            standardDeviation.set(0, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            if (swerveResult.getBestTarget().getFiducialId() == getClosestTagId(getMt2Pose2d())) {
                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                        swerveResult.getTimestampSeconds());
                            }
                        } else {
                            standardDeviation.set(0, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                    swerveResult.getTimestampSeconds());
                        }
                        // Pose2d poseWithoutAngle = new Pose2d(robotPose.toPose2d().getTranslation(),
                        // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
                    }
                }
            }

            var backLeftResult = peripherals.getBackLeftReefCamResult();
            Optional<EstimatedRobotPose> backLeftMultiTagResult = backLeftPhotonPoseEstimator.update(backLeftResult);
            if (backLeftMultiTagResult.isPresent()) {
                if (backLeftResult.getBestTarget().getPoseAmbiguity() < 0.3
                        && backLeftResult.getBestTarget().fiducialId != 5
                        && backLeftResult.getBestTarget().fiducialId != 4
                        && backLeftResult.getBestTarget().fiducialId != 14
                        && backLeftResult.getBestTarget().fiducialId != 15
                        && backLeftResult.getBestTarget().fiducialId != 3
                        && backLeftResult.getBestTarget().fiducialId != 16) {
                    Pose3d robotPose = backLeftMultiTagResult.get().estimatedPose;
                    int numFrontTracks = backLeftResult.getTargets().size();
                    Pose3d tagPose = aprilTagFieldLayout.getTagPose(backLeftResult.getBestTarget().getFiducialId())
                            .get();
                    double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                    if (distToTag < 3.2) {
                        if (inReefInteractionState()) {
                            standardDeviation.set(0, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            if (backLeftResult.getBestTarget().getFiducialId() == getClosestTagId(getMt2Pose2d())) {
                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                        backLeftResult.getTimestampSeconds());
                            }
                        } else {
                            standardDeviation.set(0, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                    backLeftResult.getTimestampSeconds());
                        }
                        // Pose2d poseWithoutAngle = new Pose2d(robotPose.toPose2d().getTranslation(),
                        // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
                    }
                }
            }

            var backRightResult = peripherals.getBackRightReefCamResult();
            Optional<EstimatedRobotPose> backRightMultiTagResult = backRightPhotonPoseEstimator.update(backRightResult);
            if (backRightMultiTagResult.isPresent()) {
                if (backRightResult.getBestTarget().getPoseAmbiguity() < 0.3
                        && backRightResult.getBestTarget().fiducialId != 5
                        && backRightResult.getBestTarget().fiducialId != 4
                        && backRightResult.getBestTarget().fiducialId != 14
                        && backRightResult.getBestTarget().fiducialId != 15
                        && backRightResult.getBestTarget().fiducialId != 3
                        && backRightResult.getBestTarget().fiducialId != 16) {
                    Pose3d robotPose = backRightMultiTagResult.get().estimatedPose;
                    int numFrontTracks = backRightResult.getTargets().size();
                    Pose3d tagPose = aprilTagFieldLayout.getTagPose(backRightResult.getBestTarget().getFiducialId())
                            .get();
                    double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                    // Logger.recordOutput("Distance to tag", distToTag);
                    if (distToTag < 3.2) {
                        if (inReefInteractionState()) {
                            standardDeviation.set(0, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    0.5
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            if (backRightResult.getBestTarget().getFiducialId() == getClosestTagId(getMt2Pose2d())) {
                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                        backRightResult.getTimestampSeconds());
                            }
                        } else {
                            standardDeviation.set(0, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(1, 0,
                                    Constants.Vision.getNumTagStdDevScalar(numFrontTracks)
                                            * Constants.Vision.getTagDistStdDevScalar(distToTag));
                            // + Math.pow(dif, Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                            // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                            standardDeviation.set(2, 0, 0.9);

                            mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                    backRightResult.getTimestampSeconds());
                        }
                        // Pose2d poseWithoutAngle = new Pose2d(robotPose.toPose2d().getTranslation(),
                        // new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())));
                    }
                }
            }
        }

        // if (isPoseInField(frontReefCamPnPPose) &&
        // !frontReefCamPnPPose.equals(defaultPose)) {
        // //
        // peripherals.setPigeonAngle(frontReefCamPnPPose.getRotation().getRadians());
        // mt2Odometry.addVisionMeasurement(frontReefCamPnPPose,
        // peripherals.getFrontReefCamLatency());
        // }

        m_currentTime = Timer.getFPGATimestamp() - m_initTime;

        m_averagedX = (m_currentX + m_averagedX) / 2;
        m_averagedY = (m_currentY + m_averagedY) / 2;
        m_averagedTheta = (m_currentTheta + m_averagedTheta) / 2;

        currentFusedOdometry[0] = m_averagedX;
        currentFusedOdometry[1] = m_averagedY;
        currentFusedOdometry[2] = m_currentTheta;
    }

    /**
     * Retrieves the states of the modules (position and ground speed) of the
     * robot's swerve drive system.
     *
     * @return An array containing the states of each wheel module, consisting of:
     *         front right module position in degrees,
     *         front right module ground speed in meters per second,
     *         front left module position in degrees,
     *         front left module ground speed in meters per second,
     *         back left module position in degrees,
     *         back left module ground speed in meters per second,
     *         back right module position in degrees,
     *         back right module ground speed in meters per second.
     */
    public double[] getModuleStates() {
        double[] states = {
                frontLeft.getCanCoderPosition() * 360.0, frontLeft.getGroundSpeed(),
                frontRight.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
                backLeft.getCanCoderPosition() * 360.0, backLeft.getGroundSpeed(),
                backRight.getCanCoderPosition() * 360.0, backRight.getGroundSpeed(),
        };
        return states;
    }

    /**
     * Retrieves the setpoints of the modules (angle and drive motors) of the
     * robot's swerve drive system.
     *
     * @return An array containing the setpoints of the angle and drive motors for
     *         each wheel module, in the order:
     *         front right angle motor, front right drive motor,
     *         front left angle motor, front left drive motor,
     *         back left angle motor, back left drive motor,
     *         back right angle motor, back right drive motor.
     */
    public double[] getModuleSetpoints() {
        double[] setpoints = {
                frontLeft.getAngleMotorSetpoint() * 360, frontLeft.getDriveMotorSetpoint(),
                frontRight.getAngleMotorSetpoint() * 360, frontRight.getDriveMotorSetpoint(),
                backLeft.getAngleMotorSetpoint() * 360, backLeft.getDriveMotorSetpoint(),
                backRight.getAngleMotorSetpoint() * 360, backRight.getDriveMotorSetpoint(),
        };
        return setpoints;
    }

    public double getRobotSpeed() {
        return (Math.abs(frontLeft.getGroundSpeed()) + Math.abs(frontRight.getGroundSpeed())
                + Math.abs(backLeft.getGroundSpeed())
                + Math.abs(backRight.getGroundSpeed())) / 4.0;
    }

    /**
     * Retrieves the current velocity of the angle motors of the robot.
     *
     * @return An array containing the current velocity of the angle motors for
     *         front right, front left, back left, and back right wheels.
     */
    public double[] getAngleMotorVelocity() {
        double[] velocity = {
                frontRight.getAngleVelocity(), frontLeft.getAngleVelocity(), backLeft.getAngleVelocity(),
                backRight.getAngleVelocity()
        };
        return velocity;
    }

    /**
     * Retrieves the current odometry information of the robot.
     *
     * @return An array containing the current X-coordinate, Y-coordinate, and
     *         orientation angle of the robot.
     */
    public double[] getOdometry() {
        double[] odometry = {
                getOdometryX(), getOdometryY(), getOdometryAngle()
        };
        return odometry;
    }

    public double[] getLocalizationOdometry() {
        double[] odometry = {
                getLocalizationOdometryX(), getLocalizationOdometryY(), getLocalizationOdometryAngle()
        };
        return odometry;
    }

    public double[] getMT2Odometry() {
        double[] odometry = {
                getMT2OdometryX(), getMT2OdometryY(), getMT2OdometryAngle()
        };
        return odometry;
    }

    private boolean autoPlacingFront = true;

    public boolean getAutoPlacementSideIsFront() {
        return autoPlacingFront;
    }

    public double getAngleDifferenceDegrees(double angle1, double angle2) {
        while (angle1 - angle2 > 180) {
            angle2 += 360;
        }
        while (angle2 - angle1 > 180) {
            angle2 -= 360;
        }
        return Math.abs(angle1 - angle2);
    }

    boolean firstTimeAutoPickup = false;
    double firstTimePickupAngle = 0.0;
    boolean firstTimeCalculated = false;
    boolean firstTimeGoingInCalculated = false;
    boolean firstTimeGoingIn = false;
    boolean hasTrack = false;

    Pose2d targetPose = new Pose2d();
    Pose2d c1 = new Pose2d();
    Pose2d c2 = new Pose2d();

    public Pose2d getGamePiecePosition() {
        if (!firstTimeAutoPickup) {
            firstTimePickupAngle = getMT2OdometryAngle();
            firstTimeAutoPickup = true;
        }

        double yaw = 0.0;
        double pitch = 0.0;
        var result = peripherals.getFrontGamePieceCamResult();
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> tracks = result.getTargets();
            for (int i = 0; i < tracks.size(); i++) {
                int id = tracks.get(i).getDetectedObjectClassID();
                if (id == 0) {
                    tracks.remove(i);
                    i--;
                }
            }
            if (tracks.isEmpty()) {
                return new Pose2d();
            } else {
                double minPitch = tracks.get(0).getPitch();
                int index = 0;
                for (int i = 1; i < tracks.size(); i++) {
                    int id = tracks.get(0).getDetectedObjectClassID();
                    java.util.logging.Logger.getGlobal().finer("id: " + id);
                    if (tracks.get(i).getPitch() < minPitch) {
                        minPitch = tracks.get(i).getPitch();
                        index = i;
                    }
                }
                pitch = minPitch;
                yaw = tracks.get(index).getYaw();
            }
        }
        // double yFromIntake = 0.0;

        if (yaw != 0.0 && pitch != 0.0) {
            java.util.logging.Logger.getGlobal().finer("calculating%");
            double cameraYaw = 15.0;
            double limelightXOffset = Constants.inchesToMeters(2.25);
            double limelightYOffset = Constants.inchesToMeters(-11.5);
            double intakeXOffset = Constants.inchesToMeters(20.5);
            double intakeYOffset = Constants.inchesToMeters(3.0); // 4.0

            double robotX = getMT2OdometryX();
            double robotY = getMT2OdometryY();
            // double robotAngle = getMT2OdometryAngle();
            double robotAngle = firstTimePickupAngle;
            Pose2d robotPose = new Pose2d(robotX, robotY, new Rotation2d(robotAngle));

            // double targetDistance = Math
            // .abs((limelightHeight - gamePieceHeight) /
            // Math.tan(Math.toRadians(-limelightAngle + pitch)));
            double targetDistance = Constants.inchesToMeters(Constants.Vision.getCoralDistanceFromPitch(pitch));
            Logger.recordOutput("Distance to Coral", targetDistance);
            double noteY = -(targetDistance * Math.sin(Math.toRadians(-cameraYaw + yaw)));
            double noteX = ((targetDistance * Math.cos(Math.toRadians(cameraYaw - yaw))));
            Logger.recordOutput("coral x", noteX);
            Logger.recordOutput("coral y", noteY);
            double xFromRobot = noteX + limelightXOffset;
            double yFromRobot = noteY + limelightYOffset;
            Pose2d coralPose = robotPose.transformBy(new Transform2d(xFromRobot, yFromRobot, new Rotation2d()));

            Logger.recordOutput("coral rc x", xFromRobot);
            Logger.recordOutput("coral rc y", yFromRobot);
            Logger.recordOutput("coral pose", coralPose);

            double xFromIntake = xFromRobot - intakeXOffset;
            double yFromIntake = yFromRobot - intakeYOffset;
            if (!firstTimeCalculated) {
                hasTrack = true;
                targetPose = robotPose.transformBy(new Transform2d(xFromIntake,
                        yFromIntake, new Rotation2d()));
                Logger.recordOutput("coral intake x", xFromIntake);
                Logger.recordOutput("coral intake y", yFromIntake);
                double projectedTx = Math.atan2(Math.tan(Math.toRadians(yaw)), Math.cos(Math.toRadians(pitch)));
                java.util.logging.Logger.getGlobal().finer("Projected: " + Math.toDegrees(projectedTx));
                java.util.logging.Logger.getGlobal().finer("actual: " + yaw);
                Pose2d intakeFieldPose = robotPose
                        .transformBy(new Transform2d(intakeXOffset, intakeYOffset, new Rotation2d()));
                double angleToPiece = Constants.standardizeAngleDegrees(Math.toDegrees(Math.atan2(
                        intakeFieldPose.getY() - coralPose.getY(), intakeFieldPose.getX() - coralPose.getX())
                        - Math.PI));
                Logger.recordOutput("Angle to Piece", (angleToPiece));
                Logger.recordOutput("Intake field pose", intakeFieldPose);

                double deltaX = coralPose.getX() - intakeFieldPose.getX();
                double deltaY = coralPose.getY() - intakeFieldPose.getY();
                double targetAngle = Math.atan2(deltaY, deltaX);
                Pose2d poseFromRobotFront = new Pose2d(
                        Constants.metersToInches(xFromRobot - Constants.inchesToMeters(16.5)),
                        Constants.metersToInches(yFromRobot), new Rotation2d());
                Logger.recordOutput("Pose from robot front", poseFromRobotFront);

                // Adjust for the robot's current heading and camera yaw
                double requiredTurn = targetAngle - Math.toRadians(cameraYaw);

                // Normalize to range [-π, π] for minimal turning
                requiredTurn = (requiredTurn + (2 * Math.PI)) % (2 * Math.PI);
                // driveToPoint(targetPose.getX(), targetPose.getY(),
                Logger.recordOutput("new angle", Math.toDegrees(requiredTurn));
                // Pose2d targetPose = robotPose
                // .transformBy(new Transform2d(xFromIntake, yFromIntake, new
                // Rotation2d(requiredTurn)));
                Logger.recordOutput("target pickup", targetPose);

                double mx = (robotX + targetPose.getX()) / 2;
                double my = (robotY + targetPose.getY()) / 2;

                // Compute the length of AB (hypotenuse)
                double abLength = Math
                        .sqrt(Math.pow(robotX - targetPose.getX(), 2) + Math.pow(robotY - targetPose.getY(), 2));

                // The expected right triangle height from midpoint to C
                double d = Math.sqrt(xFromIntake * xFromIntake + yFromIntake * yFromIntake - (abLength * abLength) / 2);

                // Compute the perpendicular direction (normalized)
                double perpX = -(robotY - targetPose.getY()) / abLength;
                double perpY = (robotX - targetPose.getX()) / abLength;

                // Compute the two possible C points
                double c1X = mx + d * perpX;
                double c1Y = my + d * perpY;

                double c2X = mx - d * perpX;
                double c2Y = my - d * perpY;
                c1 = new Pose2d(c1X, c1Y, new Rotation2d(robotAngle));
                c2 = new Pose2d(c2X, c2Y, new Rotation2d(robotAngle));
                Logger.recordOutput("c1x", c1);
                Logger.recordOutput("c1y", c2);
                firstTimeCalculated = true;
                java.util.logging.Logger.getGlobal().finer("Running once");
            }
            // targetPose.getRotation().getRadians());
            // if (){

            // } else
            // if (Math.abs(yFromIntake) < 0.2) {
            java.util.logging.Logger.getGlobal().finer("going in");
            // if (!firstTimeGoingInCalculated) {
            // firstTimeCalculated = false;
            // }
            firstTimeGoingIn = true;
            return targetPose;
            // } else if (yFromIntake < 0.0 && !firstTimeGoingIn) {
            // System.out.println("to the left");
            // return c1;
            // } else {
            // System.out.println("to the right");
            // return c2;
            // }
            // double pieceXFromIntake = noteX - intakeXOffset;
            // double pieceYFromIntake = noteY - intakeYOffset;
        } else

        {
            // System.out.println("default game piece");
            return new Pose2d();
        }
    }

    public void goToCoral() {
        double yaw = 0.0;
        double pitch = 0.0;
        var result = peripherals.getFrontGamePieceCamResult();
        List<PhotonTrackedTarget> tracks = new ArrayList<>(result.getTargets());
        List<PhotonTrackedTarget> coralTargets = new ArrayList<>();
        for (PhotonTrackedTarget track : tracks) {
            if (track.objDetectId == 1) {
                coralTargets.add(track);
            }
        }

        if (result.hasTargets() && !coralTargets.isEmpty()) {
            PhotonTrackedTarget bestTrack = coralTargets.get(0);
            yaw = bestTrack.getYaw();
            pitch = bestTrack.getPitch();
        }

        if (yaw != 0.0 && pitch != 0.0) {
            double currentAngle = -yaw;
            java.util.logging.Logger.getGlobal().finer("currentAngle: " + currentAngle);
            double wantedAngleInFrame = 14.7;
            double cameraToRobotAngleOffset = 35.0;
            rotatePID.setSetPoint(wantedAngleInFrame);
            rotatePID.updatePID(currentAngle);
            double r = -rotatePID.getResult();
            double driveAngleDeg = wantedAngleInFrame + cameraToRobotAngleOffset;
            double wantedSpeedMPS = 1.0;
            Vector v = new Vector(Math.cos(Math.toRadians(driveAngleDeg)), Math.sin(Math.toRadians(driveAngleDeg)))
                    .scaled(wantedSpeedMPS);
            autoRobotCentricDrive(v, r);
        }
    }

    public Pose2d getMt2Pose2d() {
        return mt2Odometry.getEstimatedPosition();
    }

    /**
     * Retrieves the current X-coordinate of the robot from fused odometry.
     *
     * @return The current X-coordinate of the robot.
     */
    public double getFusedOdometryX() {
        return currentFusedOdometry[0];
    }

    /**
     * Retrieves the current Y-coordinate of the robot from fused odometry.
     *
     * @return The current Y-coordinate of the robot.
     */
    public double getFusedOdometryY() {
        return currentFusedOdometry[1];
    }

    /**
     * Retrieves the current orientation angle of the robot from fused odometry.
     *
     * @return The current orientation angle of the robot in radians.
     */
    public double getFusedOdometryTheta() {
        return currentFusedOdometry[2];
    }

    /**
     * Retrieves the current X-coordinate of the robot from odometry.
     *
     * @return The current X-coordinate of the robot.
     */
    public double getOdometryX() {
        return m_odometry.getEstimatedPosition().getX();
    }

    public double getLocalizationOdometryX() {
        return loggingOdometry.getEstimatedPosition().getX();
    }

    public double getMT2OdometryX() {
        return mt2Odometry.getEstimatedPosition().getX();
    }

    /**
     * Retrieves the current Y-coordinate of the robot from odometry.
     *
     * @return The current Y-coordinate of the robot.
     */
    public double getOdometryY() {
        return m_odometry.getEstimatedPosition().getY();
    }

    public double getLocalizationOdometryY() {
        return loggingOdometry.getEstimatedPosition().getY();
    }

    public double getMT2OdometryY() {
        return mt2Odometry.getEstimatedPosition().getY();
    }

    /**
     * Retrieves the current orientation angle of the robot from odometry.
     *
     * @return The current orientation angle of the robot in radians.
     */
    public double getOdometryAngle() {
        return m_odometry.getEstimatedPosition().getRotation().getRadians();
    }

    public double getLocalizationOdometryAngle() {
        return loggingOdometry.getEstimatedPosition().getRotation().getRadians();
    }

    public double getMT2OdometryAngle() {
        return mt2Odometry.getEstimatedPosition().getRotation().getRadians();
    }

    /**
     * Drives the robot with alignment adjustment based on the specified angle from
     * placement.
     * 
     * @param degreesFromPlacement The angle in degrees from the placement
     *                             orientation to align with.
     */
    public void driveAutoAligned(double degreesFromPlacement) {
        updateOdometryFusedArray();

        double turn = degreesFromPlacement;

        double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

        if (Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if (Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.Physical.TOP_SPEED;
        double ySpeed = yPower * Constants.Physical.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);
        if (getFieldSide().equals("red")) {
            controllerVector.setI(-xSpeed);
            controllerVector.setJ(-ySpeed);
        }

        frontLeft.drive(controllerVector, turn, pigeonAngle);
        frontRight.drive(controllerVector, turn, pigeonAngle);
        backLeft.drive(controllerVector, turn, pigeonAngle);
        backRight.drive(controllerVector, turn, pigeonAngle);
    }

    /**
     * Turns the robot in robot-centric mode.
     * 
     * @param turn The rate at which the robot should turn in radians per second.
     */
    public void autoRobotCentricTurn(double turn) {
        frontLeft.drive(new Vector(0, 0), turn, 0.0);
        frontRight.drive(new Vector(0, 0), turn, 0.0);
        backLeft.drive(new Vector(0, 0), turn, 0.0);
        backRight.drive(new Vector(0, 0), turn, 0.0);
        updateOdometryFusedArray();
    }

    /**
     * Drives the robot in robot-centric mode using velocity vector and turning
     * rate.
     * 
     * @param velocityVector    The velocity vector containing x and y velocities in
     *                          meters per second (m/s).
     * @param turnRadiansPerSec The rate at which the robot should spin in radians
     *                          per second.
     */
    public void autoRobotCentricDrive(Vector velocityVector, double turnRadiansPerSec) {
        updateOdometryFusedArray();
        frontLeft.drive(velocityVector, turnRadiansPerSec, 0);
        frontRight.drive(velocityVector, turnRadiansPerSec, 0);
        backLeft.drive(velocityVector, turnRadiansPerSec, 0);
        backRight.drive(velocityVector, turnRadiansPerSec, 0);
    }

    /**
     * Drives the robot during teleoperation.
     * 
     * @apiNote This method updates the fused odometry array and controls the
     *          robot's movement based on joystick inputs.
     */
    public void teleopDrive() {
        double oiRX = -OI.getDriverRightX();
        double oiLX = OI.getDriverLeftX();
        double oiRY = OI.getDriverRightY();
        double oiLY = OI.getDriverLeftY();
        if (OI.operatorLT.getAsBoolean() && OI.operatorRT.getAsBoolean()) {
            oiRX = OI.getOperatorRightX();
            oiLX = OI.getOperatorLeftX();
            oiRY = OI.getOperatorRightY();
            oiLY = OI.getOperatorLeftY();
        }

        updateOdometryFusedArray();
        double turnLimit = 0.17;

        if (OI.driverController.getRightTriggerAxis() > 0.2 || OI.getDriverRB()) {
            // activate slowy spin
            turnLimit = 0.1;
            oiRX = oiRX * 0.8;
            oiLX = oiLX * 0.8;
            oiRY = oiRY * 0.8;
            oiLY = oiLY * 0.8;
        }
        double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
        double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
        double turn = turnLimit
                * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

        if (Math.abs(turn) < 0.05) {
            turn = 0.0;
        }
        angleSetpoint = peripherals.getPigeonAngle();
        double compensation = peripherals.getPigeonAngularVelocityW() * 0.050;
        angleSetpoint += compensation;
        // Logger.recordOutput("setpoint", angleSetpoint);
        turningPID.setSetPoint(angleSetpoint);
        double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.Physical.TOP_SPEED;
        double ySpeed = yPower * Constants.Physical.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);
        if (getFieldSide().equals("red")) {
            controllerVector.setI(-xSpeed);
            controllerVector.setJ(-ySpeed);
        }
        frontLeft.drive(controllerVector, turn, pigeonAngle);
        frontRight.drive(controllerVector, turn, pigeonAngle);
        backLeft.drive(controllerVector, turn, pigeonAngle);
        backRight.drive(controllerVector, turn, pigeonAngle);
        // }
    }

    public void robotCentricDrive(double angle) {
        double oiRX = OI.getDriverRightX();
        double oiLX = OI.getDriverLeftX();
        double oiRY = OI.getDriverRightY();
        double oiLY = OI.getDriverLeftY();
        double speedMultiplier = (((60 - Constants.metersToInches(elevator.getElevatorPosition())) * 0.4 / 50) + 0.6);

        if (elevator.getElevatorPosition() > Constants.inchesToMeters(10)) {
            oiRX = oiRX * speedMultiplier;
            oiLX = oiLX * speedMultiplier;
            oiRY = oiRY * speedMultiplier;
            oiLY = oiLY * speedMultiplier;
        }

        // Logger.recordOutput("Adjusted Right X", oiRX);
        // Logger.recordOutput("Adjusted Left X", oiLX);
        // Logger.recordOutput("Adjusted Right Y", oiRY);
        // Logger.recordOutput("Adjusted Left Y", oiLY);

        updateOdometryFusedArray();
        double turnLimit = 0.17;
        // 0.35 before

        if (OI.driverController.getRightTriggerAxis() > 0.2) {
            // activate slowy spin
            turnLimit = 0.1;
            oiRX = oiRX * 0.5;
            oiLX = oiLX * 0.5;
            oiRY = oiRY * 0.5;
            oiLY = oiLY * 0.5;
        }

        // this is correct, X is forward in field, so originalX should be the y on the
        // // joystick
        // double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(),
        // OI.getDriverLeftY()));
        // double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(),
        // OI.getDriverLeftX()));
        double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
        double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
        // if (Math.abs(originalX) < 0.005) {
        // originalX = 0;
        // }
        // if (Math.abs(originalY) < 0.005) {
        // originalY = 0;
        // }

        // double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() *
        // OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) *
        // (Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
        double turn = turnLimit
                * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

        if (Math.abs(turn) < 0.05) {
            turn = 0.0;
        }

        // if (turn == 0.0 && Timer.getFPGATimestamp() - teleopInitTime > 2.0) {
        // turningPID.setSetPoint(angleSetpoint);
        // double yaw = peripherals.getPigeonAngle();
        // while (Math.abs(angleSetpoint - yaw) > 180) {
        // if (angleSetpoint - yaw > 180) {
        // yaw += 360;
        // } else {
        // yaw -= 360;
        // }
        // }
        // double result = -1 * turningPID.updatePID(yaw);
        // Logger.recordOutput("result", result);
        // driveAutoAligned(result);
        // } else {
        angleSetpoint = peripherals.getPigeonAngle();
        double compensation = peripherals.getPigeonAngularVelocityW() * 0.050;
        angleSetpoint += compensation;
        // Logger.recordOutput("setpoint", angleSetpoint);
        turningPID.setSetPoint(angleSetpoint);
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.Physical.TOP_SPEED;
        double ySpeed = yPower * Constants.Physical.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);
        if (getFieldSide().equals("red")) {
            controllerVector.setI(-xSpeed);
            controllerVector.setJ(-ySpeed);
        }
        frontLeft.drive(controllerVector, turn, Math.toRadians(angle));
        frontRight.drive(controllerVector, turn, Math.toRadians(angle));
        backLeft.drive(controllerVector, turn, Math.toRadians(angle));
        backRight.drive(controllerVector, turn, Math.toRadians(angle));
        // }
    }

    public void teleopDriveToPiece(double yToPiece) {
        updateOdometryFusedArray();
        double turnLimit = 0.17;
        double kP = 0.8;
        // 0.35 before

        if (OI.driverController.getLeftBumper()) {
            // activate speedy spin
            // turnLimit = 1; //TODO: find a different keybind for this
        }

        // this is correct, X is forward in field, so originalX should be the y on the
        // joystick
        double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = yToPiece * kP;

        if (Math.abs(originalX) < 0.075) {
            originalX = 0;
        }

        double turn = turnLimit
                * (OI.getDriverRightX() * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

        if (Math.abs(turn) < 0.15) {
            turn = 0.0;
        }

        if (turn == 0.0) {
            turningPID.setSetPoint(angleSetpoint);
            double yaw = peripherals.getPigeonAngle();
            while (Math.abs(angleSetpoint - yaw) > 180) {
                if (angleSetpoint - yaw > 180) {
                    yaw += 360;
                } else {
                    yaw -= 360;
                }
            }
            double result = -2 * turningPID.updatePID(yaw);
            // Logger.recordOutput("result", result);
            updateOdometryFusedArray();

            double x = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
            double y = yToPiece * kP;

            if (Math.abs(originalX) < 0.05) {
                originalX = 0;
            }

            double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
            double xPower = getAdjustedX(x, y);
            double yPower = getAdjustedY(x, y);

            double xSpeed = xPower * Constants.Physical.TOP_SPEED;
            double ySpeed = yPower * Constants.Physical.TOP_SPEED;

            Vector controllerVector = new Vector(xSpeed, ySpeed);
            if (getFieldSide().equals("red")) {
                controllerVector.setI(-xSpeed);
                controllerVector.setJ(-ySpeed);
            }
            frontLeft.drive(controllerVector, result, pigeonAngle);
            frontRight.drive(controllerVector, result, pigeonAngle);
            backLeft.drive(controllerVector, result, pigeonAngle);
            backRight.drive(controllerVector, result, pigeonAngle);
        } else {
            angleSetpoint = peripherals.getPigeonAngle();
            double compensation = peripherals.getPigeonAngularVelocityW() * 0.050;
            angleSetpoint += compensation;
            // Logger.recordOutput("setpoint", angleSetpoint);
            turningPID.setSetPoint(angleSetpoint);
            double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
            double xPower = getAdjustedX(originalX, originalY);
            double yPower = getAdjustedY(originalX, originalY);

            double xSpeed = xPower * Constants.Physical.TOP_SPEED;
            double ySpeed = yPower * Constants.Physical.TOP_SPEED;

            Vector controllerVector = new Vector(xSpeed, ySpeed);
            if (getFieldSide().equals("red")) {
                controllerVector.setI(-xSpeed);
                controllerVector.setJ(-ySpeed);
            }
            frontLeft.drive(controllerVector, turn, pigeonAngle);
            frontRight.drive(controllerVector, turn, pigeonAngle);
            backLeft.drive(controllerVector, turn, pigeonAngle);
            backRight.drive(controllerVector, turn, pigeonAngle);
        }
    }

    private int hitNumber = 0;
    private int hitNumberSemiGenerous = 0;
    private int hitNumberGenerous = 0;
    private int hitNumberUltraGenerous = 0;

    public boolean hitSetPoint(double x, double y, double theta) { // adjust for l4 TODO:
        // Logger.recordOutput("Error for setpoint",
        // Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2)));
        // System.out.println("X Y error: "
        // + Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2))
        // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
        // Math.toDegrees(getMT2OdometryAngle()))
        // + " Hits: "
        // + hitNumber);
        if (Math
                .sqrt(Math.pow((x - getMT2OdometryX()), 2)
                        + Math.pow((y - getMT2OdometryY()), 2)) < 0.045
                && getAngleDifferenceDegrees(Math.toDegrees(theta),
                        Math.toDegrees(getMT2OdometryAngle())) < 1.5) {
            hitNumber += 1;
        } else {
            hitNumber = 0;
        }
        if (hitNumber > 1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean hitSetPointSemiGenerous(double x, double y, double theta) { // adjust for l4 TODO:
        // Logger.recordOutput("Error for setpoint",
        // Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2)));
        // System.out.println("X Y error: "
        // + Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2))
        // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
        // Math.toDegrees(getMT2OdometryAngle()))
        // + " Hits: "
        // + hitNumber);

        Logger.recordOutput("Error for semi-generous", Math
                .sqrt(Math.pow((x - getMT2OdometryX()), 2)
                        + Math.pow((y - getMT2OdometryY()), 2)));
        if (Math
                .sqrt(Math.pow((x - getMT2OdometryX()), 2)
                        + Math.pow((y - getMT2OdometryY()), 2)) < 0.05
                && getAngleDifferenceDegrees(Math.toDegrees(theta),
                        Math.toDegrees(getMT2OdometryAngle())) < 2) {
            hitNumberSemiGenerous += 1;
        } else {
            hitNumberSemiGenerous = 0;
        }
        if (hitNumberSemiGenerous > 3) {
            return true;
        } else {
            return false;
        }
    }

    public boolean hitSetPointGenerous(double x, double y, double theta) { // adjust for l4 TODO:
        // Logger.recordOutput("Error for setpoint",
        // Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2)));
        // System.out.println("X Y error: "
        // + Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2))
        // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
        // Math.toDegrees(getMT2OdometryAngle()))
        // + " Hits: "
        // + hitNumber);
        if (Math
                .sqrt(Math.pow((x - getMT2OdometryX()), 2)
                        + Math.pow((y - getMT2OdometryY()), 2)) < 0.10
                && getAngleDifferenceDegrees(Math.toDegrees(theta),
                        Math.toDegrees(getMT2OdometryAngle())) < 2.5) {
            hitNumberGenerous += 1;
        } else {
            hitNumberGenerous = 0;
        }
        if (hitNumberGenerous > 3) {
            return true;
        } else {
            return false;
        }
    }

    public boolean hitSetPointUltraGenerous(double x, double y, double theta) { // adjust for l4 TODO:
        // Logger.recordOutput("Error for setpoint",
        // Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2)));
        // System.out.println("X Y error: "
        // + Math.sqrt(Math.pow((x - getMT2OdometryX()), 2)
        // + Math.pow((y - getMT2OdometryY()), 2))
        // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
        // Math.toDegrees(getMT2OdometryAngle()))
        // + " Hits: "
        // + hitNumber);
        if (Math
                .sqrt(Math.pow((x - getMT2OdometryX()), 2)
                        + Math.pow((y - getMT2OdometryY()), 2)) < 0.10
                && getAngleDifferenceDegrees(Math.toDegrees(theta),
                        Math.toDegrees(getMT2OdometryAngle())) < 10.0) {
            hitNumberUltraGenerous += 1;
        } else {
            hitNumberUltraGenerous = 0;
        }
        if (hitNumberUltraGenerous > 2) {
            return true;
        } else {
            return false;
        }
    }

    public void driveToPoint(double x, double y, double theta) {
        double[] goal = { x, y, theta };
        Logger.recordOutput("Goal X, Y, Theta", goal);
        // Logger.recordOutput("Magnitude Error Inches",
        // Constants.metersToInches(Math.sqrt(Math.pow(x - getMT2OdometryX(), 2) +
        // Math.pow(y - getMT2OdometryY(), 2))));
        // Logger.recordOutput("Theta Error Degrees", Math.toDegrees(theta -
        // getMT2OdometryAngle()));

        while (Math.abs(theta - getMT2OdometryAngle()) > Math.PI) {
            if (theta - getMT2OdometryAngle() > Math.PI) {
                theta -= 2 * Math.PI;
            } else {
                theta += 2 * Math.PI;
            }
        }

        double xVelNoFF = 0.0;
        double yVelNoFF = 0.0;
        double thetaVelNoFF = 0.0;

        if (DriverStation.isAutonomousEnabled() && systemState.equals(DriveState.L4_REEF)) {
            xxPID4A.setSetPoint(x);
            yyPID4A.setSetPoint(y);
            thetaaPID4A.setSetPoint(theta);

            xxPID4A.updatePID(getMT2OdometryX());
            yyPID4A.updatePID(getMT2OdometryY());
            thetaaPID4A.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPID4A.getResult();
            yVelNoFF = yyPID4A.getResult();
            // double velmag = Math.hypot(xVelNoFF, yVelNoFF);
            // double maxvel = 0.1;
            // if (velmag > maxvel) {
            // xVelNoFF = xVelNoFF * maxvel / velmag;
            // yVelNoFF = yVelNoFF * maxvel / velmag;
            // }
            thetaVelNoFF = -thetaaPID4A.getResult();
        } else if (OI.driverPOVRight.getAsBoolean()) {
            xxPID4.setSetPoint(x);
            yyPID4.setSetPoint(y);
            thetaaPID4.setSetPoint(theta);

            xxPID4.updatePID(getMT2OdometryX());
            yyPID4.updatePID(getMT2OdometryY());
            thetaaPID4.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPID4.getResult();
            yVelNoFF = yyPID4.getResult();
            thetaVelNoFF = -thetaaPID4.getResult();

        } else if (DriverStation.isTeleopEnabled()
                && (OI.driverPOVLeft.getAsBoolean() || OI.driverPOVDown.getAsBoolean())) {

            xxPID23.setSetPoint(x);
            yyPID23.setSetPoint(y);
            thetaaPID23.setSetPoint(theta);

            xxPID23.updatePID(getMT2OdometryX());
            yyPID23.updatePID(getMT2OdometryY());
            thetaaPID23.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPID23.getResult();
            yVelNoFF = yyPID23.getResult();
            thetaVelNoFF = -thetaaPID23.getResult();

        } else if (DriverStation.isTeleopEnabled() && OI.driverPOVUp.getAsBoolean()) {

            xxPID1.setSetPoint(x);
            yyPID1.setSetPoint(y);
            thetaaPID1.setSetPoint(theta);

            xxPID1.updatePID(getMT2OdometryX());
            yyPID1.updatePID(getMT2OdometryY());
            thetaaPID1.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPID1.getResult();
            yVelNoFF = yyPID1.getResult();
            thetaVelNoFF = -thetaaPID1.getResult();

        } else if (systemState.equals(DriveState.PIECE_PICKUP)) {

            xxPIDPickup.setSetPoint(x);
            yyPIDPickup.setSetPoint(y);
            thetaaPIDPickup.setSetPoint(theta);

            xxPIDPickup.updatePID(getMT2OdometryX());
            yyPIDPickup.updatePID(getMT2OdometryY());
            thetaaPIDPickup.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPIDPickup.getResult();
            yVelNoFF = yyPIDPickup.getResult();
            thetaVelNoFF = -thetaaPIDPickup.getResult();

        } else {

            xxPID.setSetPoint(x);
            yyPID.setSetPoint(y);
            thetaaPID.setSetPoint(theta);

            xxPID.updatePID(getMT2OdometryX());
            yyPID.updatePID(getMT2OdometryY());
            thetaaPID.updatePID(getMT2OdometryAngle());

            xVelNoFF = xxPID.getResult();
            yVelNoFF = yyPID.getResult();
            thetaVelNoFF = -thetaaPID.getResult();
        }

        // double feedForwardX = targetPoint.getDouble("x_velocity") *
        // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
        // double feedForwardY = targetPoint.getDouble("y_velocity") *
        // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
        // double feedForwardTheta = -targetPoint.getDouble("angular_velocity") *
        // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;

        double finalX = xVelNoFF;
        double finalY = yVelNoFF;
        double finalTheta = thetaVelNoFF;
        // if (m_fieldSide == "blue") {
        // finalX = -finalX;
        // finalTheta = -finalTheta;
        // }
        Number[] velocityArray = new Number[] {
                finalX,
                -finalY,
                finalTheta,
        };

        Vector velocityVector = new Vector();
        double desiredThetaChange = 0;
        velocityVector.setI(velocityArray[0].doubleValue());
        velocityVector.setJ(velocityArray[1].doubleValue());
        desiredThetaChange = velocityArray[2].doubleValue();

        autoDrive(velocityVector, desiredThetaChange);

    }

    public void driveToXTheta(double x, double theta) {
        java.util.logging.Logger.getGlobal().finer(theta + "");
        // theta = Math.toRadians(theta);
        while (Math.abs(theta - getMT2OdometryAngle()) > Math.PI) {
            if (theta - getMT2OdometryAngle() > Math.PI) {
                theta -= 2 * Math.PI;
            } else {
                theta += 2 * Math.PI;
            }
        }
        xxPID.setSetPoint(x);
        thetaaPID.setSetPoint(theta);

        xxPID.updatePID(getMT2OdometryX());
        thetaaPID.updatePID(getMT2OdometryAngle());

        double xVelNoFF = xxPID.getResult();
        double yVelNoFF = OI.getDriverLeftX() * 2.9;
        double thetaVelNoFF = -thetaaPID.getResult();
        double finalX = xVelNoFF;
        double finalY = yVelNoFF;
        double finalTheta = thetaVelNoFF;
        Number[] velocityArray = new Number[] {
                finalX,
                -finalY,
                finalTheta,
        };

        Vector velocityVector = new Vector();
        double desiredThetaChange = 0;
        if (getFieldSide().equals("red")) {
            velocityVector.setI(velocityArray[0].doubleValue());
            velocityVector.setJ(-velocityArray[1].doubleValue());
        } else {
            velocityVector.setI(velocityArray[0].doubleValue());
            velocityVector.setJ(velocityArray[1].doubleValue());
        }
        desiredThetaChange = velocityArray[2].doubleValue();

        autoDrive(velocityVector, desiredThetaChange);
    }

    public void driveOnLine(Vector lineVector, Translation2d pointOnLine, double angrad) {
        Logger.recordOutput("Point On Line", new Pose2d(pointOnLine, new Rotation2d()));
        Logger.recordOutput("LineVector",
                new Pose2d(pointOnLine.plus(new Translation2d(lineVector.getI(), lineVector.getJ())),
                        new Rotation2d()));
        double oiLY = OI.getDriverLeftY();
        double oiLX = OI.getDriverLeftX();
        double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
        double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.Physical.TOP_SPEED;
        double ySpeed = yPower * Constants.Physical.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);
        if (getFieldSide().equals("red")) {
            controllerVector.setI(-ySpeed);
            controllerVector.setJ(xSpeed);
        }
        Vector projectedJoystick = lineVector.projectOther(controllerVector);
        Logger.recordOutput("controllerProjection",
                new Pose2d(new Translation2d(projectedJoystick.getI() + getMT2OdometryX(),
                        projectedJoystick.getJ() + getMT2OdometryY()), new Rotation2d(angrad)));
        Translation2d currentPoint = new Translation2d(getMT2OdometryX(), getMT2OdometryY());
        Translation2d closestPointOnLine = lineVector.getClosestPointOnLine(pointOnLine, currentPoint);
        Logger.recordOutput("closestPointOnLine", new Pose2d(closestPointOnLine, new Rotation2d(angrad)));
        xxPID.setSetPoint(closestPointOnLine.getX());
        yyPID.setSetPoint(closestPointOnLine.getY());
        while (Math.abs(angrad - getMT2OdometryAngle()) > Math.PI) {
            if (angrad - getMT2OdometryAngle() > Math.PI) {
                angrad -= 2 * Math.PI;
            } else {
                angrad += 2 * Math.PI;
            }
        }
        thetaaPID.setSetPoint(angrad);
        double toPointXVel = xxPID.updatePID(getMT2OdometryX());
        double toPointYVel = -yyPID.updatePID(getMT2OdometryY());
        double thetaVel = -thetaaPID.updatePID(getMT2OdometryAngle());
        Logger.recordOutput("feederAngle", Math.toDegrees(angrad));
        Logger.recordOutput("thetaVel", thetaVel);
        Logger.recordOutput("toPointYVel", toPointYVel);
        Logger.recordOutput("toPointXVel", toPointXVel);
        Vector toPointVector = new Vector(toPointXVel, toPointYVel);
        Vector driveVector = toPointVector.add(projectedJoystick);
        Logger.recordOutput("driveVector", new Pose2d(new Translation2d(driveVector.getI() + getMT2OdometryX(),
                driveVector.getJ() + getMT2OdometryY()), new Rotation2d(angrad)));
        autoDrive(driveVector, thetaVel);
    }

    public void driveToTheta(double theta) {
        while ((Math.toDegrees(getMT2OdometryAngle()) - theta) > 180) {
            theta += 360;
        }

        while ((theta - Math.toDegrees(getMT2OdometryAngle())) > 180) {
            theta -= 360;
        }

        // Logger.recordOutput("Drive Angle Setpoint", theta);
        turningPID.setSetPoint(theta);
        turningPID.updatePID(Math.toDegrees(getMT2OdometryAngle()));

        double result = -turningPID.getResult();
        if (Math.abs(Math.toDegrees(getMT2OdometryAngle()) - theta) < 2) {
            result = 0;
        }
        driveAutoAligned(result);
    }

    /**
     * Runs autonomous driving by providing velocity vector and turning rate.
     * 
     * @param vector            The velocity vector containing xy velocities.
     * @param turnRadiansPerSec The rate at which the robot should spin in radians
     *                          per second.
     */
    public void autoDrive(Vector vector, double turnRadiansPerSec) {
        updateOdometryFusedArray();

        double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        frontLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
        frontRight.drive(vector, turnRadiansPerSec, pigeonAngle);
        backLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
        backRight.drive(vector, turnRadiansPerSec, pigeonAngle);
    }

    /**
     * Retrieves the current velocity vector of the robot in field coordinates.
     * The velocity vector is calculated based on the individual wheel speeds and
     * orientations.
     *
     * @return The current velocity vector of the robot in meters per second (m/s).
     */
    public Vector getRobotVelocityVector() {
        Vector velocityVector = new Vector(0, 0);
        double pigeonAngleRadians = Math.toRadians(this.peripherals.getPigeonAngle());

        double frV = this.frontRight.getGroundSpeed();
        double frTheta = this.frontRight.getWheelPosition() + pigeonAngleRadians;
        double frVX = frV * Math.cos(frTheta);
        double frVY = frV * Math.sin(frTheta);
        double flV = this.frontLeft.getGroundSpeed();
        double flTheta = this.frontLeft.getWheelPosition() + pigeonAngleRadians;
        double flVX = flV * Math.cos(flTheta);
        double flVY = flV * Math.sin(flTheta);
        double blV = this.backLeft.getGroundSpeed();
        double blTheta = this.backLeft.getWheelPosition() + pigeonAngleRadians;
        double blVX = blV * Math.cos(blTheta);
        double blVY = blV * Math.sin(blTheta);
        double brV = this.backRight.getGroundSpeed();
        double brTheta = this.backRight.getWheelPosition() + pigeonAngleRadians;
        double brVX = brV * Math.cos(brTheta);
        double brVY = brV * Math.sin(brTheta);

        velocityVector.setI(frVX + flVX + blVX + brVX);
        velocityVector.setJ(frVY + flVY + blVY + brVY);

        // System.out.println("VVector: <" + velocityVector.getI() + ", " +
        // velocityVector.getJ() + ">");

        return velocityVector;
    }

    /**
     * Retrieves the acceleration vector of the robot.
     * The acceleration is calculated based on the linear acceleration measured by
     * the Pigeon IMU.
     * The acceleration is expressed in field-centric coordinates.
     *
     * @return The acceleration vector of the robot in meters per second squared
     *         (m/s^2).
     */
    public Vector getRobotAccelerationVector() {
        Vector accelerationVector = new Vector();

        Vector robotCentricAccelerationVector = this.peripherals.getPigeonLinAccel();
        double accelerationMagnitude = Constants.getDistance(robotCentricAccelerationVector.getI(),
                robotCentricAccelerationVector.getJ(), 0, 0);
        accelerationVector.setI(accelerationMagnitude * Math.cos(Math.toRadians(this.peripherals.getPigeonAngle())));
        accelerationVector.setJ(accelerationMagnitude * Math.sin(Math.toRadians(this.peripherals.getPigeonAngle())));

        return accelerationVector;
    }

    /**
     * Retrieves the path point closest to the specified time from the given path.
     * If the specified time is before the first path point, the first point is
     * returned.
     * If the specified time is after the last path point, the last point is
     * returned.
     *
     * @param path The array containing path points, each represented as a
     *             JSONArray.
     * @param time The time for which the closest path point is required.
     * @return The closest path point to the specified time.
     */
    public JSONArray getPathPoint(JSONArray path, double time) {
        for (int i = 0; i < path.length() - 1; i++) {
            JSONArray currentPoint = path.getJSONArray(i + 1);
            JSONArray previousPoint = path.getJSONArray(i);
            double currentPointTime = currentPoint.getDouble(0);
            double previousPointTime = previousPoint.getDouble(0);
            if (time >= previousPointTime && time < currentPointTime) {
                return currentPoint;
            }
        }
        if (time < path.getJSONArray(0).getDouble(0)) {
            return path.getJSONArray(0);
        } else {
            return path.getJSONArray(path.length() - 1);
        }
    }

    public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
            JSONArray pathPoints, boolean fullSend, boolean accurate) {
        JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
        int targetIndex = pathPoints.length() - 1;
        if (this.m_fieldSide == "blue") {
            currentX = Constants.Physical.FIELD_LENGTH - currentX;
            currentY = Constants.Physical.FIELD_WIDTH - currentY;
            currentTheta = Math.PI + currentTheta;
        }

        if (OI.isProcessorSide()) {
            currentY = Constants.Physical.FIELD_WIDTH - currentY;
            currentTheta = -currentTheta;
        }

        for (int i = currentIndex; i < pathPoints.length(); i++) {
            JSONObject point = pathPoints.getJSONObject(i);
            double targetX = point.getDouble("x"), targetY = point.getDouble("y"),
                    targetTheta = point.getDouble("angle"), targetXvel = point.getDouble("x_velocity"),
                    targetYvel = point.getDouble("y_velocity"), targetThetavel = point.getDouble("angular_velocity");
            while (Math.abs(targetTheta - currentTheta) > Math.PI) {
                if (targetTheta - currentTheta > Math.PI) {
                    targetTheta -= 2 * Math.PI;
                } else if (targetTheta - currentTheta < -Math.PI) {
                    targetTheta += 2 * Math.PI;
                }
            }
            double linearVelMag = Math.hypot(targetYvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    targetXvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
            double targetVelMag = Math.hypot(linearVelMag,
                    targetThetavel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
            double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
                    : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
                            + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;// If full send mode is enabled, use the full
                                                                          // send lookahead
            double deltaX = (currentX - targetX), deltaY = (currentY - targetY),
                    deltaTheta = (currentTheta - targetTheta);
            if (!insideRadius(deltaX / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    deltaY / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    deltaTheta / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
                    lookaheadRadius)) {
                targetIndex = i;
                targetPoint = pathPoints.getJSONObject(i);
                break;
            }
        }
        double targetX = targetPoint.getDouble("x"), targetY = targetPoint.getDouble("y"),
                targetTheta = targetPoint.getDouble("angle");

        while (Math.abs(targetTheta - currentTheta) > Math.PI) {
            if (targetTheta - currentTheta > Math.PI) {
                targetTheta -= 2 * Math.PI;
            } else if (targetTheta - currentTheta < -Math.PI) {
                targetTheta += 2 * Math.PI;
            }
        }
        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);

        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);
        double pidScaler = 1;
        double xVelNoFF = xPID.getResult() * pidScaler;
        double yVelNoFF = yPID.getResult() * pidScaler;
        double thetaVelNoFF = -thetaPID.getResult();
        double f = (accurate ? Constants.Autonomous.ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY
                : Constants.Autonomous.FEED_FORWARD_MULTIPLIER);
        double feedForwardX = targetPoint.getDouble("x_velocity") * f;
        double feedForwardY = targetPoint.getDouble("y_velocity") * f;
        double feedForwardTheta = -targetPoint.getDouble("angular_velocity") * f * 0.1;

        double finalX = xVelNoFF + feedForwardX;
        double finalY = yVelNoFF + feedForwardY;
        double finalTheta = (thetaVelNoFF + feedForwardTheta) * 1.25;
        if (m_fieldSide == "blue") {
            finalX = -finalX;
            finalY = -finalY;
        }

        if (OI.isProcessorSide()) {
            finalY = -finalY;
            finalTheta = -finalTheta;
        }

        Number[] velocityArray = new Number[] {
                finalX,
                -finalY,
                finalTheta,
                targetIndex,
        };
        double linearVelMag = Math.hypot(
                targetPoint.getDouble("x_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                targetPoint.getDouble("y_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
        double targetVelMag = Math.hypot(linearVelMag,
                targetPoint.getDouble("angular_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
        double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
                : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
                        + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;

        Logger.recordOutput("x-vel", xVelNoFF);
        Logger.recordOutput("y-vel", yVelNoFF);
        Logger.recordOutput("theta-vel", thetaVelNoFF);
        Logger.recordOutput("wanted-theta-vel",
                targetPoint.getDouble("angular_velocity"));
        Logger.recordOutput("FF-theta-vel", feedForwardTheta);
        Logger.recordOutput("FF-x-vel", feedForwardX);
        Logger.recordOutput("FF-y-vel", feedForwardY);
        Logger.recordOutput("current point idx", currentIndex);
        Logger.recordOutput("point idx", velocityArray[3].intValue());
        Logger.recordOutput("look-ahead", lookaheadRadius);
        Logger.recordOutput("target-point", new Pose2d(targetX, targetY, new Rotation2d(targetTheta)));
        Logger.recordOutput("Velocity Array",
                new double[] { finalX, -finalY, finalTheta });
        Logger.recordOutput("dx", targetX - currentX);
        Logger.recordOutput("dy", targetY - currentY);
        Logger.recordOutput("dtheta", targetTheta - currentTheta);
        return velocityArray;
    }

    public boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
        // Logger.recordOutput("deltax", deltaX);
        // Logger.recordOutput("deltay", deltaY);
        // Logger.recordOutput("deltaTheta", deltaTheta);
        Logger.recordOutput("Error", Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)));
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
    }

    public void calculateAngleChange(double angle) {
        double pigeonAngleDegrees = this.peripherals.getPigeonAngle();
        double targetAngle = 0;
        if (getFieldSide() == "red") {
            targetAngle = angle + 180;
        } else {
            targetAngle = angle;
        }

        if (DriverStation.isAutonomousEnabled() && getFieldSide() == "red") {
            pigeonAngleDegrees = 180 + pigeonAngleDegrees;
        }
        this.turningPID.setSetPoint(Constants.standardizeAngleDegrees(targetAngle));
        this.turningPID.updatePID(Constants.standardizeAngleDegrees(pigeonAngleDegrees));
        double turnResult = -turningPID.getResult();

        this.driveAutoAligned(turnResult);
    }

    private DriveState handleStateTransition() {
        switch (wantedState) {
            case DEFAULT:
                return DriveState.DEFAULT;
            case IDLE:
                return DriveState.IDLE;
            case REEF:
                return DriveState.REEF;
            case REEF_MORE:
                return DriveState.REEF_MORE;
            case BACK:
                return DriveState.BACK;
            case L3_REEF:
                return DriveState.L3_REEF;
            case L4_REEF:
                return DriveState.L4_REEF;
            case ALGAE:
                return DriveState.ALGAE;
            case ALGAE_MORE:
                return DriveState.ALGAE_MORE;
            case ALGAE_MORE_MORE:
                return DriveState.ALGAE_MORE_MORE;
            case PROCESSOR:
                return DriveState.PROCESSOR;
            case PROCESSOR_MORE:
                return DriveState.PROCESSOR_MORE;
            case NET:
                return DriveState.NET;
            case NET_MORE:
                return DriveState.NET_MORE;
            case FEEDER:
                return DriveState.FEEDER;
            case SCORE_L23:
                return DriveState.SCORE_L23;
            case AUTO_FEEDER:
                return DriveState.AUTO_FEEDER;
            case FEEDER_ALIGN:
                return DriveState.FEEDER_ALIGN;
            case AUTO_L1:
                return DriveState.AUTO_L1;
            case AUTO_L1_MORE:
                return DriveState.AUTO_L1_MORE;
            case FEEDER_AUTO:
                return DriveState.FEEDER_AUTO;
            case PIECE_PICKUP:
                return DriveState.PIECE_PICKUP;
            case AUTO_CLIMB:
                return DriveState.AUTO_CLIMB;
            case STOP:
                return DriveState.STOP;
            case MOVE_TO_POINT:
                return DriveState.MOVE_TO_POINT;
            default:
                return DriveState.IDLE;
        }
    }

    Vector scoreL23Vector = new Vector(2.5, 0);
    double[] l23Setpoint = { 0, 0, 0 };

    Vector pickupAlgaeFrontVector = new Vector(2.5, 0);
    double[] algaeSetpoint = { 0, 0, 0 };
    Vector pickupAlgaeBackVector = new Vector(-2.5, 0);

    public double getDistanceFromL23Setpoint() {
        return Math.hypot(Math.abs(getMT2OdometryX() - l23Setpoint[0]), Math.abs(getMT2OdometryY() - l23Setpoint[1]));
    }

    public double getDistanceFromAlgaeSetpoint() {
        return Math.hypot(Math.abs(getMT2OdometryX() - algaeSetpoint[0]),
                Math.abs(getMT2OdometryY() - algaeSetpoint[1]));
    }

    public double getThetaToCenterReef() {
        double theta = 0.0;
        if (OI.isRedSide()) {
            theta = Math.atan2(Constants.Reef.centerRed.getY() - getMT2OdometryY(),
                    Constants.Reef.centerRed.getX() - getMT2OdometryX());
        } else {
            theta = Math.atan2((Constants.Reef.centerBlue.getY() - getMT2OdometryY()),
                    (Constants.Reef.centerBlue.getX() - getMT2OdometryX()));
        }
        if (getAngleDifferenceDegrees(Math.toDegrees(theta), Math.toDegrees(getMT2OdometryAngle())) > 90.0) {
            theta -= Math.PI;
        }
        return theta;
    }

    public double[] getClosestPoint(double[] lineStart, double[] lineEnd) { // chatGPT ahh code
        // System.out.println("X1 - X2; " + (lineStart[0] - lineEnd[0]) + "Y1 - Y2" +
        // (lineStart[1] - lineEnd[1]));
        double x1 = lineStart[0];
        double y1 = lineStart[1];
        double x2 = lineEnd[0];
        double y2 = lineEnd[1];
        double px = getMT2OdometryX();
        double py = getMT2OdometryY();

        double dx = x2 - x1;
        double dy = y2 - y1;
        double lenSq = dx * dx + dy * dy;

        if (lenSq == 0) {
            return new double[] { x1, y1 };
        }

        double t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
        t = Math.max(0, Math.min(1, t));

        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        return new double[] { closestX, closestY };
    }

    public double getThetaToPoint(double xMeters, double yMeters) {
        return Math.atan2(yMeters - getMT2OdometryY(),
                xMeters - getMT2OdometryX());
    }

    public boolean isL1Face() { // returns true if the robot is by a face of the L1, returns false if the robot
                                // is around the point on the edge of l1 faces
        if (isOnBlueSide()) {

        } else {

        }
        return true; // TODO: theoretically this should work because the closest
    }

    public Pose2d getClosestPose(Pose2d pose1, Pose2d pose2) {
        double dist1 = Math.hypot(Math.abs(pose1.getX() - getMT2OdometryX()),
                Math.abs(pose1.getY() - getMT2OdometryY()));
        double dist2 = Math.hypot(Math.abs(pose2.getX() - getMT2OdometryX()),
                Math.abs(pose2.getY() - getMT2OdometryY()));
        if (dist1 <= dist2) {
            return pose1;
        } else {
            return pose2;
        }
    }

    public double[] origionalSetpointPose = { 0.0, 0.0, 0.0 };
    public boolean firstTimeReef = true;

    public double getNetXSetpoint() {
        if (isOnBlueSide()) {
            return Constants.Reef.netBlueXM;
        } else {
            return Constants.Reef.netRedXM;
        }
    }

    public double getNetXMoreSetpoint() {
        if (isOnBlueSide()) {
            return Constants.Reef.netBlueXMore;
        } else {
            return Constants.Reef.netRedXMore;
        }
    }

    public double getNetThetaSetpoint() {
        if (isOnBlueSide()) {
            if (autoPlacingFront) {
                return Constants.Reef.netBlueFrontThetaR;
            } else {
                return Constants.Reef.netBlueBackThetaR;
            }
        } else {
            if (autoPlacingFront) {
                return Constants.Reef.netRedFrontThetaR;
            } else {
                return Constants.Reef.netRedBackThetaR;
            }
        }
    }

    public boolean isOnBlueSide() {
        if (getMT2OdometryX() < Constants.Physical.FIELD_LENGTH / 2.0) {
            return true;
        } else {
            return false;
        }
    }

    public double[] getNetXTheta() {
        double goalX = getMT2OdometryX();
        double goalTheta = getMT2OdometryAngle();

        if (isOnBlueSide()) {
            goalX = Constants.Reef.netBlueXM;
            if (getAngleDifferenceDegrees(Math.toDegrees(getMT2OdometryAngle()),
                    Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
                autoPlacingFront = true;
                goalTheta = Constants.Reef.netBlueFrontThetaR;
            } else {
                autoPlacingFront = false;
                goalTheta = Constants.Reef.netBlueBackThetaR;
            }
        } else {
            goalX = Constants.Reef.netRedXM;
            if (getAngleDifferenceDegrees(Math.toDegrees(getMT2OdometryAngle()),
                    Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
                autoPlacingFront = true;
                goalTheta = Constants.Reef.netRedFrontThetaR;
            } else {
                autoPlacingFront = false;
                goalTheta = Constants.Reef.netRedBackThetaR;
            }
        }

        double[] xTheta = { goalX, goalTheta };
        return xTheta;
    }

    public double[] getNetMoreXTheta() {
        double goalX = getMT2OdometryX();
        double goalTheta = getMT2OdometryAngle();

        if (isOnBlueSide()) {
            goalX = Constants.Reef.netBlueXMore;
            if (getAngleDifferenceDegrees(Math.toDegrees(getMT2OdometryAngle()),
                    Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
                autoPlacingFront = true;
                goalTheta = Constants.Reef.netBlueFrontThetaR;
            } else {
                autoPlacingFront = false;
                goalTheta = Constants.Reef.netBlueBackThetaR;
            }
        } else {
            goalX = Constants.Reef.netRedXMore;
            if (getAngleDifferenceDegrees(Math.toDegrees(getMT2OdometryAngle()),
                    Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
                autoPlacingFront = true;
                goalTheta = Constants.Reef.netRedFrontThetaR;
            } else {
                autoPlacingFront = false;
                goalTheta = Constants.Reef.netRedBackThetaR;
            }
        }

        double[] xTheta = { goalX, goalTheta };
        return xTheta;
    }

    public double distanceFromCenterOfReef() {
        if (isOnBlueSide()) {
            return Math.hypot((getMT2OdometryX() - Constants.Reef.centerBlue.getX()),
                    ((getMT2OdometryY() - Constants.Reef.centerBlue.getY())));
        } else {
            return Math.hypot((getMT2OdometryX() - Constants.Reef.centerRed.getX()),
                    ((getMT2OdometryY() - Constants.Reef.centerRed.getY())));
        }
    }

    private final Vector backupVector = new Vector(-20.0, 0.0);
    private final Vector otherBackupVector = new Vector(20.0, 0.0);

    @Override
    public void periodic() {
        Logger.recordOutput("Robot Velocity", getRobotSpeed());
        // Pose2d target = getGamePiecePosition();
        // System.out.println(Math.toDegrees(getThetaToCenterReef()));
        // Translation2d t1 = new Translation2d(getMT2OdometryX(), getMT2OdometryY());
        // Rotation2d r1 = new Rotation2d(getThetaToCenterReef());
        // Pose2d p1 = new Pose2d(t1, r1);
        // Logger.recordOutput("L1 Auto Angle", p1);
        // Logger.recordOutput("Robot Odometry", getMT2Odometry());
        updateOdometryFusedArray();
        // process inputs
        DriveState newState = handleStateTransition();
        double[] setpoint = { 0.0, 0.0, 0.0 };
        double standardizedAngle = Constants.standardizeAngleDegrees(Math.toDegrees(getMT2OdometryAngle()));
        if (newState != systemState) {
            systemState = newState;
        }
        Logger.recordOutput("Drive State", systemState);
        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = DriveState.DEFAULT;
        }

        if (!systemState.equals(DriveState.AUTO_CLIMB)) {
            firstClimb = false;
        }
        if (!systemState.equals(DriveState.PIECE_PICKUP)) {
            firstTimeAutoPickup = false;
            firstTimeCalculated = false;
            firstTimeGoingInCalculated = false;
            firstTimeGoingIn = false;
            hasTrack = false;
        }
        if (!OI.getDriverA()) {
            firstTimeReef = true;
        }
        // Logger.recordOutput("L1 Vector",
        // new Pose2d(new Translation2d(2.0, 2.0),
        // new Rotation2d(
        // Math.atan2(getClosestL1Points().get(0)[1] - getClosestL1Points().get(
        // 1)[1], getClosestL1Points().get(0)[0] - getClosestL1Points().get(1)[0]))));

        // Logger.recordOutput("Point 1", new Pose2d(new
        // Translation2d(getClosestL1Points().get(0)[0],
        // getClosestL1Points().get(0)[1]), new Rotation2d(0.0)));
        // Logger.recordOutput("Point 2", new Pose2d(new
        // Translation2d(getClosestL1Points().get(1)[0],
        // getClosestL1Points().get(1)[1]), new Rotation2d(0.0)));

        // Pose2d l1Pose = new Pose2d(getClosestL1PointXY()[0],
        // getClosestL1PointXY()[1],
        // new Rotation2d(getThetaToPoint(getClosestL1PointXY()[0],
        // getClosestL1PointXY()[1])));

        switch (systemState) {
            case DEFAULT:
                teleopDrive();
                break;
            case MOVE_TO_POINT:
                driveToPoint(1, 1, 0);
                break;
            default:
                teleopDrive();
                break;
        }
        Logger.recordOutput("Robot Position", new Pose2d(new Translation2d(getFusedOdometryX(), getFusedOdometryY()),
                new Rotation2d(getFusedOdometryTheta())));
    }

    /**
     * Calculates the adjusted y-coordinate based on the original x and y
     * coordinates.
     *
     * @param originalX The original x-coordinate.
     * @param originalY The original y-coordinate.
     * @return The adjusted y-coordinate.
     */
    public double getAdjustedY(double originalX, double originalY) {
        double adjustedY = originalY * Math.sqrt((1 - (Math.pow(originalX, 2)) / 2));
        return adjustedY;
    }

    /**
     * Calculates the adjusted x-coordinate based on the original x and y
     * coordinates.
     *
     * @param originalX The original x-coordinate.
     * @param originalY The original y-coordinate.
     * @return The adjusted x-coordinate.
     */
    public double getAdjustedX(double originalX, double originalY) {
        double adjustedX = originalX * Math.sqrt((1 - (Math.pow(originalY, 2)) / 2));
        return adjustedX;
    }
}