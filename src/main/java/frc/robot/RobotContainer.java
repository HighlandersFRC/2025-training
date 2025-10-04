package frc.robot;

import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveTrainOverride;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetRobotStateSimpleOnce;
import frc.robot.commands.Test;
import frc.robot.commands.ZeroPigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Straightenator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public final Peripherals peripherals = new Peripherals();
  public final Elevator elevator = new Elevator();
  public final Straightenator straightenator = new Straightenator();
  public final Arm arm = new Arm();
  public final Manipulator manipulator = new Manipulator();
  public final Intake intake = new Intake();

  public final Drive drive = new Drive(peripherals, elevator);
  public final Superstructure superstructure = new Superstructure(drive, peripherals, elevator, straightenator, arm,
      manipulator, intake);

  File[] autoFiles = new File[Constants.Autonomous.paths.length];
  Command[] autos = new Command[Constants.Autonomous.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];
  private Command autonomousCommand;

  public boolean algaeMode;
  public boolean manualMode;

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

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
      put("Note in Robot", () -> true);
    }
  };

  public RobotContainer() {
    configureBindings();
    for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
      try {
        autoFiles[i] = new File(
            Filesystem.getDeployDirectory().getPath() + "/"
                + Constants.Autonomous.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0)
            .getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, peripherals, commandMap,
            conditionMap);
        java.util.logging.Logger.getGlobal()
            .info("Loaded Path: " + Constants.Autonomous.paths[i]);
      } catch (Exception e) {
        java.util.logging.Logger.getGlobal()
            .severe("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":"
                + e);
      }
    }
  }

  Command auto;

  private void configureBindings() {
    OI.driverPOVRight
        .whileTrue(
            new ConditionalCommand(
                new SetRobotStateSimple(superstructure, SuperState.NET),
                new SetRobotStateSimple(superstructure, SuperState.AUTO_L4_PLACE),
                () -> algaeMode))
        .onFalse(
            new ConditionalCommand(
                new SetRobotStateSimple(superstructure, SuperState.L4_SCORE),
                new InstantCommand(),
                () -> manualMode));

    OI.driverRT
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.INTAKING));

    OI.driverLT
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.OUTTAKE));

    OI.driverPOVLeft
        .whileTrue(
            new ConditionalCommand(
                new SetRobotState(superstructure, SuperState.ALGAE_LOW),
                new SetRobotStateSimple(superstructure, SuperState.AUTO_L2_PLACE),
                () -> algaeMode))
        .onFalse(
            new ConditionalCommand(
                new SetRobotStateSimple(superstructure, SuperState.L2_SCORE),
                new InstantCommand(),
                () -> manualMode));

    OI.driverPOVDown
        .whileTrue(
            new ConditionalCommand(
                new SetRobotState(superstructure, SuperState.ALGAE_HIGH),
                new SetRobotStateSimple(superstructure, SuperState.AUTO_L3_PLACE),
                () -> algaeMode))
        .onFalse(
            new ConditionalCommand(
                new SetRobotStateSimple(superstructure, SuperState.L3_SCORE),
                new InstantCommand(),
                () -> manualMode));

    OI.driverPOVUp
        .whileTrue(
            new ConditionalCommand(
                new SetRobotState(superstructure, SuperState.PROCESSOR),
                new SetRobotState(superstructure, SuperState.AUTO_L1_SCORE),
                () -> algaeMode));

    OI.driverMenuButton
        .whileTrue(new ZeroPigeon(peripherals));

    OI.driverLB
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.DEFAULT));

    OI.driverPOVLeft
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.DEFAULT));
  }

  public Command getAutonomousCommand() {
    int selectedPath = Constants.Autonomous.getSelectedPathIndex();
    if (selectedPath >= Constants.Autonomous.paths.length) {
      selectedPath = -1;
    }
    if (selectedPath == -1) {
      java.util.logging.Logger.getGlobal().info("Selected Path: None");
      return new DoNothing();
    } else {
      this.drive.autoInit(autoPoints[selectedPath]);
      java.util.logging.Logger.getGlobal()
          .info("Selected Path: " + Constants.Autonomous.paths[selectedPath]);
      return this.autos[selectedPath];
    }
  }
}