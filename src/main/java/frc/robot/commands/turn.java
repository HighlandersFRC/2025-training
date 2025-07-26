package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;
import frc.robot.tools.math.PID;

public class turn extends Command {
  private final Drive driveSubsystem;
  private final PID yawPID = new PID(0.1, 0.0, 0.0); 

  public turn(Drive drive) {
    driveSubsystem = drive;

    yawPID.setMinInput(-180);
    yawPID.setMaxInput(180);
    yawPID.setContinuous(true);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    yawPID.setSetPoint(Constants.angle); 
  }

  @Override
  public void execute() {
    double currentTheta = driveSubsystem.getAngle();
    double turnOut = -yawPID.updatePID(currentTheta); 
    driveSubsystem.autoDrive(new Vector(0, 0), turnOut);
    System.out.println(yawPID.getError());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(yawPID.getError()) < 0.05;
  }
}
