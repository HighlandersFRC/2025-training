// package frc.robot;

// import java.io.IOException;
// import java.util.List;

// import org.littletonrobotics.junction.LogFileUtil;
// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.commands.DriveToPoint;
// import frc.robot.commands.FollowPath;
// import frc.robot.commands.pathing.FFTest;
// import frc.robot.commands.pathing.FinalPathing;
// import frc.robot.commands.pathing.FollowTest;
// import frc.robot.commands.pathing.NewPathing;
// import frc.robot.commands.pathing.FollowTest;
// import frc.robot.commands.pathing.testPathing;
// import frc.robot.commands.pathing.testPathingFF;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Peripherals;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Drive.DriveState;
// import frc.robot.subsystems.Superstructure.SuperState;
// import frc.robot.tools.PathLoader;
// import frc.robot.tools.PathLoaderOld;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.math.Vector;

// public class Robot extends LoggedRobot {
//     private final RobotContainer m_robotContainer;
//     private final Drive drive;
//     private final Peripherals peripherals;
//     private final Superstructure superstructure;
//     private double setAngle = 0;
//     private Command m_autonomousCommand;
//     PathLoader path = new PathLoader();
//     List<PosePoint> autoPath;
//     List<frc.robot.tools.PathLoaderOld.PosePoint> Path;

//     public Robot() {

//         Logger.recordMetadata("ChassisBot", "Chassis");

//         if (isReal()) {
//             Logger.addDataReceiver(new NT4Publisher());
//         } else {
//             setUseTiming(false);
//             String logPath = LogFileUtil.findReplayLog();
//             Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a
//                                                                                                   // new log
//         }

//         Logger.start();
//         m_robotContainer = new RobotContainer();
//         drive = m_robotContainer.drive;
//         peripherals = m_robotContainer.peripherals;
//         superstructure = m_robotContainer.superstructure;

//     }

//     @Override
//     public void robotPeriodic() {
//         CommandScheduler.getInstance().run();
//     }

//     @Override
//     public void robotInit() {
//         try {
//             autoPath = PathLoader.loadPath("opmovethenturn.polarpath");
//             Path = PathLoaderOld.loadPath("testingsquares.polarpath");
//         } catch (IOException e) {
//             e.printStackTrace();
//         }
//     }

//     @Override
//     public void disabledInit() {


//     }

//     @Override
//     public void disabledPeriodic() {
//     }

//     @Override
//     public void autonomousInit() {
//        new NewPathing(autoPath, drive).schedule();
//       // new FollowTest(Path, drive).schedule();
//     }

//     @Override
//     public void autonomousPeriodic() {
//     }

//     @Override
//     public void teleopInit() {
//         if (m_autonomousCommand != null) {
//             m_autonomousCommand.cancel();
//         }
//         superstructure.setWantedState(SuperState.DEFAULT);

//     }

//     @Override
//     public void teleopPeriodic() {
//         superstructure.periodic();
        
//     }

//     @Override
//     public void testInit() {
//         CommandScheduler.getInstance().cancelAll();
//     }

//     @Override
//     public void testPeriodic() {
//     }

//     @Override
//     public void simulationInit() {
//     }

//     @Override
//     public void simulationPeriodic() {
//     }
// }
package frc.robot;

import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveTrainOverride;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.Test;
import frc.robot.commands.pathing.FinalPathing;
import frc.robot.commands.pathing.NewPathing;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Drive.WANTED_GAME_PIECE;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Straightenator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.PathLoader;
import frc.robot.tools.PathLoader.PosePoint;
import frc.robot.tools.PathLoaderOld;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
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
    private final Straightenator straightenator;
    private final Manipulator manipulator;
    private double setAngle = 0;
    private Command m_autonomousCommand;
    PathLoader path = new PathLoader();
    List<PosePoint> autoPath;
    List<frc.robot.tools.PathLoaderOld.PosePoint> Path;
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
         manipulator = m_robotContainer.manipulator;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {
        elevator.init();
        try {
             autoPath = PathLoader.loadAuto("opauto.polarauto");
             Path = PathLoaderOld.loadPath("testingsquares.polarpath");
                    } 
             catch (IOException e) {
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
        new FinalPathing(autoPath, drive).schedule();
        // try {]\[
        
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
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        superstructure.setWantedState(SuperState.DEFAULT);

    }

    @Override
    public void teleopPeriodic() {

        if (OI.driverLT.getAsBoolean()) {
            straightenator.setWantedState(Straightenator.StraightenatorState.DEFAULT);
        } else {
            straightenator.setWantedState(Straightenator.StraightenatorState.IDLE);
        }

        if(OI.driverA.getAsBoolean()){
            manipulator.setWantedState(Manipulator.ManipulatorState.CORAL_INTAKE);
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