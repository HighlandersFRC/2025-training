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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveTrainOverride;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.Test;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;

    private Command m_autonomousCommand;
    String m_fieldSide = "blue";

    File[] autoFiles;
    Command[] autos;
    JSONObject[] autoJSONs;
    JSONArray[] autoPoints;
    SendableChooser<String> fieldSideChooser = new SendableChooser<String>();

    PathLoader path = new PathLoader();
    JSONObject autoPath;
    PolarAutoFollower autoCommand;

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

    boolean bPressed = false;
    boolean bManualPressed = false;
    HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
        {
            put("Note in Robot", () -> true);
        }
    };

    public Robot() {
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
    }

    @Override
    public void robotPeriodic() {
        Constants.periodic();
        CommandScheduler.getInstance().run();

        m_robotContainer.superstructure.algaeMode = m_robotContainer.algaeMode;
        m_robotContainer.drive.algaeMode = m_robotContainer.algaeMode;
    }

    @Override
    public void robotInit() {
        Constants.init();
        m_robotContainer.elevator.init();
        m_robotContainer.drive.init(m_fieldSide);
        m_robotContainer.arm.init();
        m_robotContainer.intake.init();
        autoFiles = new File[Constants.paths.size()];
        autos = new Command[Constants.paths.size()];
        autoJSONs = new JSONObject[Constants.paths.size()];
        autoPoints = new JSONArray[Constants.paths.size()];
        for (int i = 0; i < Constants.paths.size(); i++) {
            try {
                autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.paths.get(i));
                FileReader scanner = new FileReader(autoFiles[i]);
                autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
                autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0)
                        .getJSONArray("sampled_points");
                autos[i] = new PolarAutoFollower(autoJSONs[i],
                        m_robotContainer.drive, m_robotContainer.peripherals, commandMap, conditionMap);
            } catch (Exception e) {
                System.out.println("ERROR LOADING PATH " + Constants.paths.get(i) + ":" + e);
            }
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
        m_robotContainer.elevator.rezeroElevator();
        double autoInitTime = Timer.getFPGATimestamp();
        m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
        if (OI.isBlueSide()) {
            java.util.logging.Logger.getGlobal().info("ON BLUE SIDE");
            m_fieldSide = "blue";
        } else {
            java.util.logging.Logger.getGlobal().info("ON RED SIDE");
            m_fieldSide = "red";
        }
        this.m_robotContainer.drive.setFieldSide(m_fieldSide);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        java.util.logging.Logger.getGlobal().info("Auto init time" + (Timer.getFPGATimestamp() - autoInitTime));
        m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_robotContainer.elevator.rezeroElevator();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.superstructure.setWantedState(SuperState.HANDOFF);

    }

    @Override
    public void teleopPeriodic() {

        if (OI.driverY.getAsBoolean()) {
            if (bManualPressed) {
                m_robotContainer.manualMode = !m_robotContainer.manualMode;
                bManualPressed = false;
            }
        } else {
            bManualPressed = true;
        }
        if (OI.driverB.getAsBoolean()) {
            if (bPressed) {
                m_robotContainer.algaeMode = !m_robotContainer.algaeMode;
                bPressed = false;
            }
        } else {
            bPressed = true;
        }
        // Logger.recordOutput("Current Mode", superstructure.getAlgaeMode());

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
        Logger.recordOutput("Algae Mode", m_robotContainer.algaeMode);
        Logger.recordOutput("Manual Mode", m_robotContainer.manualMode);
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