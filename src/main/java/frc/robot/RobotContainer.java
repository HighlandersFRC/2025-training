package frc.robot;

import frc.robot.commands.Print;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Superstructure;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autos.PolarAutoFollower;

public class RobotContainer {
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public final Drive drive = new Drive();
  public final Peripherals peripherals = new Peripherals();
  public final Superstructure superstructure = new Superstructure(drive, peripherals);
  private Command autonomousCommand;

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("Print", () -> new Print());
    }
  };

  public RobotContainer() {
    configureBindings();

  }

  Command auto;

  private void configureBindings() {
    String pathName = ".polarauto";
    try {
      File file = new File(Filesystem.getDeployDirectory(), pathName);
      JSONObject json = new JSONObject(new JSONTokener(new FileReader(file)));

      Command autoCommand = new PolarAutoFollower(json, commandMap, null);
      this.auto = autoCommand;

      System.out.println("Loaded Path: " + pathName);
    } catch (Exception e) {
      System.out.println("ERROR LOADING PATH " + pathName + ": " + e.getMessage());
    }

  }

  public Command getAutonomousCommand() {
    return this.auto;
  }
}
