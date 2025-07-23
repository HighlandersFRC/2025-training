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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.PathLoader;
import frc.robot.tools.PathLoader.PosePoint;
import frc.robot.tools.math.Vector;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private final Drive drive;
    private final Peripherals peripherals;
    private final Superstructure superstructure;
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
        superstructure = m_robotContainer.superstructure;

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
        superstructure.setWantedState(SuperState.DEFAULT);

    }

    @Override
    public void teleopPeriodic() {
        superstructure.periodic();
        if (OI.driverB.getAsBoolean()) {
            superstructure.setWantedState(SuperState.PATH_TO_POINT);
        }
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