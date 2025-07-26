package frc.robot;

import frc.robot.commands.Print;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Peripherals;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public final Drive drive = new Drive();
  public final Peripherals peripherals = new Peripherals();
  private Command autonomousCommand;

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("Print", () -> new Print());
    }
  };

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // add trigger-to-command mappings here
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
