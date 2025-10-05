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

  File[] autoFiles;
  Command[] autos;
  JSONObject[] autoJSONs;
  JSONArray[] autoPoints;
  private Command autonomousCommand;

  public boolean algaeMode;
  public boolean manualMode;

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("L4_Place", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L4_PLACE));
      put("L3_Place", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L3_PLACE));
      put("L2_Place", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L2_PLACE));
      put("L1_Score", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L1_SCORE));
      put("Net", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.NET));
      put("L4_Score", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L4_SCORE));
      put("L3_Score", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L3_SCORE));
      put("L2_Score", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.L2_SCORE));
      put("Reset", () -> new SetRobotStateSimpleOnce(superstructure, SuperState.DEFAULT));
      put("Zero", () -> new ZeroPigeon(peripherals));
    }
  };

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
      put("Note in Robot", () -> true);
    }
  };

  public RobotContainer() {
    configureBindings();
    autoFiles = new File[Constants.paths.size()];
    autos = new Command[Constants.paths.size()];
    autoJSONs = new JSONObject[Constants.paths.size()];
    autoPoints = new JSONArray[Constants.paths.size()];
    for (int i = 0; i < Constants.paths.size(); i++) {
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.paths.get(i));
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, peripherals, commandMap, conditionMap);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH " + Constants.paths.get(i) + ":" + e);
      }
    }
  }

  Command auto;

  private void configureBindings() {

    OI.driverRT
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.INTAKING))
        .onFalse(new SetRobotStateSimple(superstructure, SuperState.INTAKE_IDLE));

    OI.driverLT
        .whileTrue(new SetRobotStateSimple(superstructure, SuperState.OUTTAKE));

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
    OI.driverX.whileTrue(new SetRobotStateSimple(superstructure, SuperState.OUTTAKE_ONESIDE));
  }

  public Command getAutonomousCommand() {
    int selectedPath = Constants.Autonomous.getSelectedPathIndex();
    System.out.println("Selected Path Index: " + selectedPath);
    if (selectedPath >= Constants.paths.size()) {
      selectedPath = -1;
    }
    if (selectedPath == -1) {
      java.util.logging.Logger.getGlobal().info("Selected Path: None");
      return new DoNothing();
    } else {
      this.drive.autoInit(autoPoints[selectedPath]);
      java.util.logging.Logger.getGlobal()
          .info("Selected Path: " + Constants.paths.get(selectedPath));
      return this.autos[selectedPath];
    }
  }
}