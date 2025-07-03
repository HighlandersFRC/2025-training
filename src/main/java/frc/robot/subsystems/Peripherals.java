package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Peripherals {
    private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
    private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();

    public Peripherals() {
    }

    /**
     * Initializes the Peripherals subsystem.
     * 
     * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
     * It also applies the default command to the Peripherals subsystem.
     */
    public void init() {
        // Set the mount pose configuration for the IMU
        pigeonConfig.MountPose.MountPosePitch = 0.3561480641365051;
        pigeonConfig.MountPose.MountPoseRoll = -0.10366992652416229;
        pigeonConfig.MountPose.MountPoseYaw = -0.24523599445819855;

        // Apply the IMU configuration
        pigeon.getConfigurator().apply(pigeonConfig);

        // Zero the IMU angle
        zeroPigeon();

        setPigeonPitchOffset(getPigeonPitch());

    }

    /**
     * Sets the IMU angle to 0
     */
    public void zeroPigeon() {
        setPigeonAngle(0.0);
    }

    /**
     * Sets the angle of the IMU
     * 
     * @param degrees - Angle to be set to the IMU
     */
    public void setPigeonAngle(double degrees) {
        pigeon.setYaw(degrees);
    }

    /**
     * Retrieves the yaw of the robot
     * 
     * @return Yaw in degrees
     */
    public double getPigeonAngle() {
        return -pigeon.getYaw().getValueAsDouble();
    }

    /**
     * Retrieves the absolute angular velocity of the IMU's Z-axis in device
     * coordinates.
     *
     * @return The absolute angular velocity of the IMU's Z-axis in device
     *         coordinates.
     *         The value is in degrees per second.
     */
    public double getPigeonAngularVelocity() {
        return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
    }

    /**
     * Retrieves the absolute angular velocity of the IMU's Z-axis in world
     * coordinates.
     *
     * @return The absolute angular velocity of the IMU's Z-axis in world
     *         coordinates.
     *         The value is in radians per second.
     */
    public double getPigeonAngularVelocityW() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    /**
     * Retrieves the acceleration vector of the robot
     * 
     * @return Current acceleration vector of the robot
     */
    public Vector getPigeonLinAccel() {
        Vector accelVector = new Vector();
        accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        return accelVector;
    }

    public double getPigeonPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getPigeonPitchAdjusted() {
        return getPigeonPitch() - pigeonPitchOffset;
    }

    double pigeonPitchOffset = 0.0;

    public void setPigeonPitchOffset(double newOffset) {
        pigeonPitchOffset = newOffset;
    }

    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    public void periodic() {

    }
}