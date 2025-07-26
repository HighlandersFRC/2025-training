// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.math;

/** Add your docs here. */
public class Derivatives {
    public double t, d0, d1, d2;

    public Derivatives(double t, double d0, double d1, double d2) {
        this.t = t;
        this.d0 = d0;
        this.d1 = d1;
        this.d2 = d2;
    }
}
