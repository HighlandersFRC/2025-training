// // package frc.robot;

// // import frc.robot.commands.PolarAutoFollower;
// // import frc.robot.commands.Test;
// // import frc.robot.commands.ZeroPigeon;
// // import frc.robot.subsystems.Drive;
// // import frc.robot.subsystems.Peripherals;
// // import frc.robot.subsystems.Superstructure;

// // import java.io.File;
// // import java.io.FileReader;
// // import java.util.HashMap;
// // import java.util.function.Supplier;
// // import java.util.stream.Stream;

// // import org.json.JSONArray;
// // import org.json.JSONObject;
// // import org.json.JSONTokener;

// // import edu.wpi.first.wpilibj.Filesystem;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// // import edu.wpi.first.wpilibj2.command.button.Trigger;

// // public class RobotContainer {
// //   public final Drive drive = new Drive();
// //   public final Peripherals peripherals = new Peripherals();
// //   public final Superstructure superstructure = new Superstructure(drive, peripherals);
// //   private Command autonomousCommand;

// //   public RobotContainer() {
// //     configureBindings();

// //   }

// //   Command auto;

// //   private void configureBindings() {
// //     OI.driverA.whileTrue(new ZeroPigeon(peripherals));
// //   }

// //   public Command getAutonomousCommand() {
// //     return this.auto;
// //   }
// // }
// package frc.robot;

// import frc.robot.commands.PolarAutoFollower;
// import frc.robot.commands.SetRobotState;
// import frc.robot.commands.Test;
// import frc.robot.commands.ZeroPigeon;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Peripherals;
// import frc.robot.subsystems.Straightenator;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Drive.WANTED_GAME_PIECE;
// import frc.robot.subsystems.Superstructure.SuperState;

// import java.io.File;
// import java.io.FileReader;
// import java.util.HashMap;
// import java.util.function.Supplier;
// import java.util.stream.Stream;

// import org.json.JSONArray;
// import org.json.JSONObject;
// import org.json.JSONTokener;

// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// public class RobotContainer {
//   public final Drive drive = new Drive();
//   public final Peripherals peripherals = new Peripherals();
//   public final Elevator elevator = new Elevator();
//   public final Straightenator straightenator = new Straightenator();
//   public final Superstructure superstructure = new Superstructure(drive, peripherals, elevator);
//   private Command autonomousCommand;

//   public RobotContainer() {
//     configureBindings();

//   }

//   Command auto;

//   private void configureBindings() {
//     OI.driverMenuButton.whileTrue(new ZeroPigeon(peripherals));

//   }

//   public Command getAutonomousCommand() {
//     return this.auto;
//   }
// }
package frc.robot;

import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.Test;
import frc.robot.commands.ZeroPigeon;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Straightenator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.WANTED_GAME_PIECE;
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
  public final Drive drive = new Drive();
  public final Peripherals peripherals = new Peripherals();
  public final Elevator elevator = new Elevator();
  public final Straightenator straightenator = new Straightenator();
  public final Manipulator manipulator = new Manipulator();
  public final Superstructure superstructure = new Superstructure(drive, peripherals, elevator, manipulator);
  private Command autonomousCommand;

  public RobotContainer() {
    configureBindings();

  }

  Command auto;

  private void configureBindings() {
    OI.driverMenuButton.whileTrue(new ZeroPigeon(peripherals));
    OI.driverA.whileTrue(new SetRobotState(superstructure, SuperState.CORAL_INTAKE));

  }

  public Command getAutonomousCommand() {
    return this.auto;
  }
}