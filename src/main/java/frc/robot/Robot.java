package frc.robot;

import java.io.IOException;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PathLoader;
import frc.robot.tools.PathLoader.PosePoint;
import frc.robot.tools.math.Vector;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private final Drive drive;
    private final Peripherals peripherals;
    private final TalonFX testMotor;
    private double setAngle = 0;
    private Command m_autonomousCommand;
    PathLoader path = new PathLoader();
    List<PosePoint> autoPath;

    public Robot() {

        Logger.recordMetadata("ChassisBot", "Chassis");

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a
                                                                                                  // new log
        }

        Logger.start();
        m_robotContainer = new RobotContainer();
        drive = m_robotContainer.drive;
        peripherals = m_robotContainer.peripherals;
        testMotor = new TalonFX(4);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {
        try {
            autoPath = PathLoader.loadPath("test.polarpath");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        new ParallelCommandGroup(new FollowPath(autoPath, drive),
                new DriveToPoint(drive))
                .schedule();

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
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

        drive.driveAuto(fieldCentricVector, rightX);

        // drive.drive(leftX, leftY, rightX, peripherals.getPigeonAngle());

        // System.out.println(angleDeg);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}