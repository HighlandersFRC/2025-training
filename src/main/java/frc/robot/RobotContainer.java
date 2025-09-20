package frc.robot;

import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetRobotStateSimpleOnce;
import frc.robot.commands.Test;
import frc.robot.commands.ZeroPigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Straightenator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

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

public class RobotContainer {
  public final Peripherals peripherals = new Peripherals();
  public final Elevator elevator = new Elevator();
  public final Straightenator straightenator = new Straightenator();
  public final Arm arm = new Arm();
  public final Manipulator manipulator = new Manipulator();

  public final Drive drive = new Drive(peripherals, elevator);
  public final Superstructure superstructure = new Superstructure(drive, peripherals, elevator, straightenator, arm,
      manipulator);

  private Command autonomousCommand;

  public RobotContainer() {
    configureBindings();

  }

  Command auto;

  private void configureBindings() {
    OI.driverMenuButton.whileTrue(new ZeroPigeon(peripherals));
    OI.driverLB.whileTrue(new SetRobotStateSimple(superstructure, SuperState.HANDOFF));
    OI.driverPOVRight.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L4_PLACE));
    OI.driverPOVDown.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L3_PLACE));
    OI.driverPOVLeft.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L2_PLACE));
    OI.driverY.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L2_SCORE));
    OI.driverB.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L3_SCORE));
    OI.driverA.whileTrue(new SetRobotStateSimple(superstructure, SuperState.AUTO_L4_SCORE));
  }

  public Command getAutonomousCommand() {
    return this.auto;
  }
}