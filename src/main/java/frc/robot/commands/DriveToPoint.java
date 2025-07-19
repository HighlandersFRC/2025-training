package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;
import frc.robot.tools.math.PID;

public class DriveToPoint extends Command {
  private final Drive driveSubsystem;

  private final PID xPID = new PID(4, 0.0, 2.1);
  private final PID yPID = new PID(4, 0.0, 2.1);
  private final PID yawPID = new PID(2, 0.0, 2.9);

  public DriveToPoint(Drive drive) {
    System.out.println("started");
    driveSubsystem = drive;

    yawPID.setMinInput(-180);
    yawPID.setMaxInput(180);
    yawPID.setContinuous(true);

    xPID.setMinOutput(-3.0);
    xPID.setMaxOutput(3.0);

    yPID.setMinOutput(-3.0);
    yPID.setMaxOutput(3.0);

    yawPID.setMinInput(-0.5);
    yawPID.setMaxOutput(0.5);
    //
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    xPID.setSetPoint(Constants.x);
    yPID.setSetPoint(Constants.y);
    yawPID.setSetPoint(Constants.angle);

    double currentX = driveSubsystem.getX();
    double currentY = driveSubsystem.getY();
    double currentTheta = driveSubsystem.getAngle();

    double xOut = -xPID.updatePID(currentX) / 2;
    double yOut = yPID.updatePID(currentY) / 2;
    double turnOut = -yawPID.updatePID(currentTheta) / 8;

    Vector relativeVector = new Vector(xOut, yOut);
    driveSubsystem.autoDrive(relativeVector, turnOut);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    boolean posClose = Math.abs(xPID.getError()) < 0.03 &&
        Math.abs(yPID.getError()) < 0.03;
    boolean angleClose = Math.abs(yawPID.getError()) < 1.0;
    return posClose && angleClose && Constants.lastPoint;
  }
}
