package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveTrainOverride;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.Test;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Straightenator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.PathLoader;
import java.io.File;
import java.io.FileReader;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONObject;
import org.json.JSONTokener;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private final Drive drive;
    private final Peripherals peripherals;
    private final Superstructure superstructure;
    private final Elevator elevator;
    private final Arm arm;
    private final Straightenator straightenator;
    private final Manipulator manipulator;
    private double setAngle = 0;
    private Command m_autonomousCommand;

    private Timer handoffTimer = new Timer();
    private boolean handoffSequenceActive = false;
    PathLoader path = new PathLoader();
    JSONObject autoPath;
    PolarAutoFollower autoCommand;

    private double[] frozenPoint = null;
    private double[] frozenPointAlgae = null;
    HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
        {
            put("Command1", () -> new Test("command1"));
            put("Command2", () -> new Test("command2"));
            put("Command3", () -> new Test("command3"));
            put("Command4", () -> new Test("command4"));
            put("Print", () -> new Test("Print"));
            put("DriveOverride", () -> new DriveTrainOverride());
        }
    };

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
        elevator = m_robotContainer.elevator;
        straightenator = m_robotContainer.straightenator;
        arm = m_robotContainer.arm;
        manipulator = m_robotContainer.manipulator;

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {
        elevator.init();
        arm.init();
        drive.init("blue");
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        String pathName = "Paths/Commands.polarauto";
        try {
            File file = new File(Filesystem.getDeployDirectory(), pathName);
            if (!file.exists()) {
                System.out.println("File not found: " + file.getAbsolutePath());
                return;
            }

            JSONObject json = new JSONObject(new JSONTokener(new FileReader(file)));
            System.out.println(json);
            autoCommand = new PolarAutoFollower(json, commandMap, null);
        } catch (Exception e) {
            System.out.println("ERROR LOADING PATH " + pathName + ": " + e.getMessage());
            e.printStackTrace();
        }
        CommandScheduler.getInstance().schedule(autoCommand);
        // try {
        // List<PathLoader.PosePoint> pathPoints =
        // PathLoader.loadPath("square.polarpath");
        // new PurePursuitAutoFollower(pathPoints, drive).schedule();
        // } catch (IOException e) {
        // e.printStackTrace();
        // }

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        arm.zeroOnEnable();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        superstructure.setWantedState(SuperState.HANDOFF);

    }

    @Override
    public void teleopPeriodic() {

        if (OI.driverRT.getAsBoolean()) {
            straightenator.setWantedState(Straightenator.StraightenatorState.DEFAULT);
        } else {
            straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
        }

        // if (OI.driverX.getAsBoolean() && !handoffSequenceActive) {
        // arm.setWantedState(ArmState.L4_SCORE);
        // manipulator.setWantedState(ManipulatorState.OUTAKE);
        // handoffTimer.restart();
        // handoffSequenceActive = true;
        // }
        // if (handoffSequenceActive && handoffTimer.hasElapsed(0.5)) {
        // arm.setWantedState(ArmState.HANDOFF);
        // superstructure.setWantedState(SuperState.HANDOFF);
        // handoffSequenceActive = false;
        // }
        // if (straightenator.isFar()) {
        // manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
        // } else if (OI.driverLT.getAsBoolean()) {
        // manipulator.setWantedState(ManipulatorState.OUTAKE);
        // }
        // OI.driverA.whileTrue(new SetRobotStateSimple(superstructure,
        // SuperState.HANDOFF));

        // if (OI.driverB.getAsBoolean()) {
        // superstructure.setWantedState(SuperState.AUTO_L4_SCORE);
        // arm.setWantedState(ArmState.L4_PLACE);
        // }
        // if (OI.driverA.getAsBoolean()) {
        // arm.setWantedState(ArmState.HANDOFF);
        // if (arm.isReadyForHandoff()) {
        // superstructure.setWantedState(SuperState.HANDOFF);
        // }
        // }
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