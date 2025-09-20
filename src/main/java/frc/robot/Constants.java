// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Test;
import frc.robot.subsystems.Superstructure.SuperState;

public final class Constants {
        public static double x = 0;
        public static double y = 0;
        public static double angle = 0;
        public static boolean lastPoint = false;

        public static final ArrayList<String> paths = new ArrayList<String>();

        public static boolean isReady = false;

        public static int getSelectedPathIndex() {
                String selectedAuto = OI.autoSendableChooser.getSelected();
                for (int i = 0; i < paths.size(); i++) {
                        if (selectedAuto.equals(paths.get(i))) {
                                return i;
                        }
                }
                return -1;
        }

        static {
                File[] autos = Filesystem.getDeployDirectory().listFiles();
                for (File file : autos) {
                        if (file.getAbsolutePath().endsWith(".polarauto")) {
                                paths.add(file.getName());
                        }

                }
        }

        public static final class Swerve {

                public static final double TURN_kP = 1;
                public static final double TURN_kI = 0;
                public static final double TURN_kD = 0;

                public static final double chassisWidthMeters = 0.66041;
                public static final double chassisLengthMeters = 0.8128;

                public static final double moduleX = chassisWidthMeters / 2;
                public static final double moduleY = chassisLengthMeters / 2;

        }

        public static final class Autonomous {
                public static final int STAGNATE_BOOST = 35;
                public static final int STAGNATE_THRESHOLD = 8;
                // lookahead distance is a function:
                // LOOKAHEAD = AUTONOMOUS_LOOKAHEAD_DISTANCE * velocity + MIN_LOOKAHEAD_DISTANCE
                // their constants
                public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 0.04; // Lookahead at 1m/s scaled by wanted
                                                                                 // velocity
                public static final double FULL_SEND_LOOKAHEAD = 0.60;
                public static final double MIN_LOOKAHEAD_DISTANCE = 0.05; // Lookahead distance at 0m/s
                // Path follower will end if within this radius of the final point
                public static final double AUTONOMOUS_END_ACCURACY = 0.40;
                public static final double ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY = 0.05;
                // When calculating the point distance, will divide x and y by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1.0;
                // When calculating the point distance, will divide theta by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;
                // Feed Forward Multiplier
                public static final double FEED_FORWARD_MULTIPLIER = 0.5;
                public static final double ACCURATE_FOLLOWER_FEED_FORWARD_MULTIPLIER = 1;
                public static final String[] paths = new String[] {
                                "2AlgaeCenter.polarauto",
                                "2+1PieceFeeder.polarauto",
                                "3PieceFeederSmart.polarauto",
                                "4PieceFeederGroundSmart.polarauto",
                                "TushPush.polarauto",
                };
                public static final double AUTO_PLACE_DISTANCE = 5;

                public static int getSelectedPathIndex() {
                        if (OI.autoChooserConnected()) {
                                if (OI.autoChooser.getRawButton(1)) {
                                        return 0;
                                }
                                if (OI.autoChooser.getRawButton(2)) {
                                        return 1;
                                }
                                if (OI.autoChooser.getRawButton(3)) {
                                        return 2;
                                }
                                if (OI.autoChooser.getRawButton(4)) {
                                        return 3;
                                }
                                if (OI.autoChooser.getRawButton(5)) {
                                        return 4;
                                }
                        } else {
                                return (int) Math.round(SmartDashboard.getNumber("ROBOT AUTO OVERIDE", -1));
                        }
                        return -1;
                }

        }

        public static void periodic() {
                int index = Autonomous.getSelectedPathIndex();
                if (index == -1 || index > Constants.Autonomous.paths.length) {
                } else {
                }
                index = getSelectedPathIndex();
                if (index == -1) {
                        Logger.recordOutput("Selected Auto", "Do Nothing");
                } else {
                        Logger.recordOutput("Selected Auto", paths.get(index));
                }
        }

        public static void init() {

                Logger.recordOutput("Reef/AlgaeBlueFrontPlacingPositions",
                                Constants.Reef.algaeBlueFrontPlacingPositions.toString());
                Logger.recordOutput("Reef/AlgaeRedFrontPlacingPositions",
                                Constants.Reef.algaeRedFrontPlacingPositions.toString());

                Logger.recordOutput("Reef/BlueFrontPlacingPositions",
                                Constants.Reef.blueFrontPlacingPositions.toString());
                Logger.recordOutput("Reef/RedFrontPlacingPositions",
                                Constants.Reef.redFrontPlacingPositions.toString());

                Logger.recordOutput("Reef/L4BlueFrontPlacingPositions",
                                Constants.Reef.l4BlueFrontPlacingPositions.toString());
                Logger.recordOutput("Reef/L4RedFrontPlacingPositions",
                                Constants.Reef.l4RedFrontPlacingPositions.toString());

                Logger.recordOutput("Reef/L3BlueFrontPlacingPositions",
                                Constants.Reef.l3BlueFrontPlacingPositions.toString());
                Logger.recordOutput("Reef/L3RedFrontPlacingPositions",
                                Constants.Reef.l3RedFrontPlacingPositions.toString());

                Logger.recordOutput("Reef/BlueFrontPlacingPositionsMore",
                                Constants.Reef.blueFrontPlacingPositionsMore.toString());
                Logger.recordOutput("Reef/RedFrontPlacingPositionsMore",
                                Constants.Reef.redFrontPlacingPositionsMore.toString());

        }

        public static class Reef {
                public static final double PERFECT_BRANCH_OFFSET_L23 = inchesToMeters(1.625);
                public static final double PERFECT_BRANCH_OFFSET_L4 = inchesToMeters(1.125);

                // positive is from face of reef towards center of reef
                // negative means futher from reef
                public static final double A_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.5);
                public static final double B_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.875);
                public static final double C_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.125);
                public static final double D_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.25);
                public static final double E_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.125);
                public static final double F_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.125);
                public static final double G_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.125);
                public static final double H_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.125);
                public static final double I_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(-0.5940);
                public static final double J_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(0.375);
                public static final double K_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(0.7157);
                public static final double L_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                                inchesToMeters(1.75);

                public static final double A_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.0);
                public static final double B_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.5);
                public static final double C_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.75);
                public static final double D_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.0);
                public static final double E_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double F_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double G_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double H_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double I_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.125);
                public static final double J_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.5);
                public static final double K_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(3.0);
                public static final double L_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(3.0);

                public static final double A_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.5);
                public static final double B_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.5);
                public static final double C_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.875);
                public static final double D_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.25);
                public static final double E_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double F_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double G_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double H_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.625);
                public static final double I_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.25);
                public static final double J_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(1.75);
                public static final double K_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.0);
                public static final double L_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                                inchesToMeters(2.0);

                // right when facing the reef side is positive
                // negative makes robot go more to the left
                public static final double A_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double B_BRANCH_OFFSET_SIDE = inchesToMeters(1.5);
                public static final double C_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double D_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double E_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double F_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double G_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double H_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double I_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                public static final double J_BRANCH_OFFSET_SIDE = inchesToMeters(-2.0);
                public static final double K_BRANCH_OFFSET_SIDE = inchesToMeters(-1.0);
                public static final double L_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);

                public static final double A_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double B_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double C_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double D_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double E_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double F_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double G_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double H_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double I_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(1.5);
                public static final double J_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(-2.0);
                public static final double K_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                public static final double L_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);

                public static final double A_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double B_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double C_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double D_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double E_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double F_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double G_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double H_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double I_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(1.5);
                public static final double J_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(-2.0);
                public static final double K_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final double L_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);

                // public static final double A_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double B_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double C_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double D_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double E_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double F_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double G_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double H_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double I_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double J_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double K_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);
                // public static final double L_BRANCH_OFFSET = PERFECT_BRANCH_OFFSET_L4 -
                // inchesToMeters(1.125);

                // public static final double A_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double B_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double C_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double D_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double E_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double F_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double G_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double H_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double I_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double J_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double K_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double L_BRANCH_OFFSET_L3 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);

                // public static final double A_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double B_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double C_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double D_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double E_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double F_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double G_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double H_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double I_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double J_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double K_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);
                // public static final double L_BRANCH_OFFSET_L2 = PERFECT_BRANCH_OFFSET_L23 -
                // inchesToMeters(1.625);

                // right when facing the reef side is positive
                // negative makes robot go more to the left
                // public static final double A_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double B_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double C_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double D_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double E_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double F_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double G_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double H_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double I_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double J_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double K_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);
                // public static final double L_BRANCH_OFFSET_SIDE = inchesToMeters(0.0);

                // public static final double A_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double B_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double C_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double D_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double E_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double F_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double G_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double H_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double I_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double J_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double K_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);
                // public static final double L_BRANCH_OFFSET_SIDE_L3 = inchesToMeters(0.0);

                // public static final double A_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double B_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double C_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double D_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double E_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double F_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double G_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double H_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double I_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double J_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double K_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                // public static final double L_BRANCH_OFFSET_SIDE_L2 = inchesToMeters(0.0);
                public static final Translation2d centerBlue = new Translation2d(inchesToMeters(176.746),
                                inchesToMeters(158.501));
                public static final Translation2d centerRed = new Translation2d(
                                Constants.Physical.FIELD_LENGTH - inchesToMeters(176.746),
                                inchesToMeters(158.501));
                public static final double faceToZoneLine = inchesToMeters(12); // Side of the reef to the inside of the
                                                                                // reef
                                                                                // zone line

                public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in
                                                                          // clockwise
                                                                          // order
                                                                          // public static final List<Map<ReefHeight,
                                                                          // Pose3d>>
                                                                          // blueBranchPositions = new
                                                                          // ArrayList<>(); // Starting at the right
                                                                          // // branch facing the driver
                                                                          // // station in clockwise
                                                                          // public static final List<Map<ReefHeight,
                                                                          // Pose3d>>
                                                                          // redBranchPositions = new
                                                                          // ArrayList<>(); // Starting at the right

                public static final double PROCESSOR_Y_OFFSET_M = inchesToMeters(50.0);
                public static final double PROCESSOR_MORE_Y_OFFSET_M = inchesToMeters(20.0);
                public static final double NET_X_OFFSET_M = inchesToMeters(50.0);
                public static final double NET_X_OFFSET_MORE = inchesToMeters(34.0);

                public static final Translation2d processorBlueFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorBlueFrontPlacingRotation = new Rotation2d(
                                degreesToRadians(270));

                public static final Translation2d processorRedFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorRedFrontPlacingRotation = new Rotation2d(degreesToRadians(90));

                public static final Translation2d processorBlueBackPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorBlueBackPlacingRotation = new Rotation2d(degreesToRadians(90));

                public static final Translation2d processorRedBackPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorRedBackPlacingRotation = new Rotation2d(degreesToRadians(270));

                public static final Translation2d processorMoreBlueFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_MORE_Y_OFFSET_M);
                public static final Rotation2d processorMoreBlueFrontPlacingRotation = new Rotation2d(
                                degreesToRadians(270));

                public static final Translation2d processorMoreRedFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_MORE_Y_OFFSET_M);
                public static final Rotation2d processorMoreRedFrontPlacingRotation = new Rotation2d(
                                degreesToRadians(90));

                public static final Translation2d processorMoreBlueBackPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_MORE_Y_OFFSET_M);
                public static final Rotation2d processorMoreBlueBackPlacingRotation = new Rotation2d(
                                degreesToRadians(90));

                public static final Translation2d processorMoreRedBackPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_MORE_Y_OFFSET_M);
                public static final Rotation2d processorMoreRedBackPlacingRotation = new Rotation2d(
                                degreesToRadians(270));

                public static final Pose2d processorBlueFrontPlacingPosition = new Pose2d(
                                processorBlueFrontPlacingTranslation,
                                processorBlueFrontPlacingRotation);
                public static final Pose2d processorRedFrontPlacingPosition = new Pose2d(
                                processorRedFrontPlacingTranslation,
                                processorRedFrontPlacingRotation);
                public static final Pose2d processorBlueBackPlacingPosition = new Pose2d(
                                processorBlueBackPlacingTranslation,
                                processorBlueBackPlacingRotation);
                public static final Pose2d processorRedBackPlacingPosition = new Pose2d(
                                processorRedBackPlacingTranslation,
                                processorRedBackPlacingRotation);

                public static final Pose2d processorMoreBlueFrontPlacingPosition = new Pose2d(
                                processorMoreBlueFrontPlacingTranslation,
                                processorMoreBlueFrontPlacingRotation);
                public static final Pose2d processorMoreRedFrontPlacingPosition = new Pose2d(
                                processorMoreRedFrontPlacingTranslation,
                                processorMoreRedFrontPlacingRotation);
                public static final Pose2d processorMoreBlueBackPlacingPosition = new Pose2d(
                                processorMoreBlueBackPlacingTranslation,
                                processorMoreBlueBackPlacingRotation);
                public static final Pose2d processorMoreRedBackPlacingPosition = new Pose2d(
                                processorMoreRedBackPlacingTranslation,
                                processorMoreRedBackPlacingRotation);

                public static final double netBlueXM = (Constants.Physical.FIELD_LENGTH / 2) - NET_X_OFFSET_M;
                public static final double netRedXM = (Constants.Physical.FIELD_LENGTH / 2) + NET_X_OFFSET_M;
                public static final double netBlueXMore = (Constants.Physical.FIELD_LENGTH / 2) - NET_X_OFFSET_MORE;
                public static final double netRedXMore = (Constants.Physical.FIELD_LENGTH / 2) + NET_X_OFFSET_MORE;

                public static final double netBlueFrontThetaR = degreesToRadians(0.0);
                public static final double netRedFrontThetaR = degreesToRadians(180.0);
                public static final double netBlueBackThetaR = degreesToRadians(180.0);
                public static final double netRedBackThetaR = degreesToRadians(0.0);

                public static final List<Pose2d> blueL1FrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> redL1FrontPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> blueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> redFrontPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> blueFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> redFrontPlacingPositionsMore = new ArrayList<>();

                public static final List<Pose2d> l4BlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l4RedFrontPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> l3BlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l3RedFrontPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> algaeBlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> algaeRedFrontPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> algaeBlueFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> algaeRedFrontPlacingPositionsMore = new ArrayList<>();

                static {
                        calculateReefPoints();
                }

                // Angle to face red side:
                // 180=AB
                // 120=CD
                // 60=EF
                // 0=GH
                // -60=IJ
                // -120=KL

                public static void calculateReefPoints() {
                        System.out.println("recalculating points");
                        blueL1FrontPlacingPositions.clear();
                        redL1FrontPlacingPositions.clear();
                        blueFrontPlacingPositions.clear();
                        redFrontPlacingPositions.clear();
                        blueFrontPlacingPositionsMore.clear();
                        redFrontPlacingPositionsMore.clear();
                        l4BlueFrontPlacingPositions.clear();
                        l4RedFrontPlacingPositions.clear();
                        l3BlueFrontPlacingPositions.clear();
                        l3RedFrontPlacingPositions.clear();
                        algaeBlueFrontPlacingPositions.clear();
                        algaeRedFrontPlacingPositions.clear();
                        algaeBlueFrontPlacingPositionsMore.clear();
                        algaeRedFrontPlacingPositionsMore.clear();
                        centerFaces[0] = new Pose2d(
                                        inchesToMeters(144.003),
                                        inchesToMeters(158.500),
                                        Rotation2d.fromDegrees(180));
                        centerFaces[1] = new Pose2d(
                                        inchesToMeters(160.373),
                                        inchesToMeters(186.857),
                                        Rotation2d.fromDegrees(120));
                        centerFaces[2] = new Pose2d(
                                        inchesToMeters(193.116),
                                        inchesToMeters(186.858),
                                        Rotation2d.fromDegrees(60));
                        centerFaces[3] = new Pose2d(
                                        inchesToMeters(209.489),
                                        inchesToMeters(158.502),
                                        Rotation2d.fromDegrees(0));
                        centerFaces[4] = new Pose2d(
                                        inchesToMeters(193.118),
                                        inchesToMeters(130.145),
                                        Rotation2d.fromDegrees(-60));
                        centerFaces[5] = new Pose2d(
                                        inchesToMeters(160.375),
                                        inchesToMeters(130.144),
                                        Rotation2d.fromDegrees(-120));

                        for (int face = 0; face < 6; face++) {
                                Pose2d l1FrontRight = new Pose2d();
                                Pose2d l1FrontLeft = new Pose2d();
                                Pose2d l2FrontRight = new Pose2d();
                                Pose2d l2FrontLeft = new Pose2d();
                                Pose2d frontRightMore = new Pose2d();
                                Pose2d frontLeftMore = new Pose2d();
                                Pose2d l3FrontRight = new Pose2d();
                                Pose2d l3FrontLeft = new Pose2d();
                                Pose2d l4FrontRight = new Pose2d();
                                Pose2d l4FrontLeft = new Pose2d();
                                Pose2d algaeFront = new Pose2d();
                                Pose2d algaeFrontMore = new Pose2d();

                                Pose2d poseDirection = new Pose2d(centerBlue,
                                                Rotation2d.fromDegrees(180 + (60 * face)));
                                double adjustX = inchesToMeters(30.738);
                                double adjustY = inchesToMeters(6.469);
                                double adjustXL1 = inchesToMeters(30.738);
                                double adjustYL1 = inchesToMeters(6.469);
                                double adjustXMore = inchesToMeters(70.738);
                                double adjustYMore = inchesToMeters(6.469);
                                double adjustAlgaeX = inchesToMeters(35.738);
                                double adjustAlgaeY = inchesToMeters(0.0);
                                double adjustAlgaeMoreX = inchesToMeters(16.738);
                                double adjustAlgaeMoreY = inchesToMeters(0.0);

                                l1FrontRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXL1,
                                                                                                adjustYL1,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L1_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L1_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXL1,
                                                                                                adjustYL1,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L1_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L1_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()
                                                                                - Math.PI));

                                l1FrontLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXL1,
                                                                                                -adjustYL1,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L1_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L1_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXL1,
                                                                                                -adjustYL1,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L1_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L1_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()
                                                                                - Math.PI));

                                algaeFront = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                algaeFrontMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                frontRightMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                if (poseDirection.getRotation().getDegrees() > 179.0
                                                && poseDirection.getRotation().getDegrees() < 181.0) {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET_L3,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET_L3,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET_L2,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + B_BRANCH_OFFSET_L2,
                                                                                                        adjustY + B_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("b branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > 119.0
                                                && poseDirection.getRotation().getDegrees() < 121.0) {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET_L3,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET_L3,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET_L2,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + L_BRANCH_OFFSET_L2,
                                                                                                        adjustY + L_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("d branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > 59.0
                                                && poseDirection.getRotation().getDegrees() < 61.0) {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET_L3,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET_L3,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET_L2,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + J_BRANCH_OFFSET_L2,
                                                                                                        adjustY + J_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("f branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > -1.0
                                                && poseDirection.getRotation().getDegrees() < 1.0) {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET_L3,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET_L3,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET_L2,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + H_BRANCH_OFFSET_L2,
                                                                                                        adjustY + H_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("h branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > -61.0
                                                && poseDirection.getRotation().getDegrees() < -59.0) {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET_L3,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET_L3,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET_L2,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + F_BRANCH_OFFSET_L2,
                                                                                                        adjustY + F_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("j branch: " + l4FrontRight);
                                } else {
                                        l4FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET_L3,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET_L3,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontRight = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET_L2,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + D_BRANCH_OFFSET_L2,
                                                                                                        adjustY + D_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("l branch: " + l4FrontRight);
                                }
                                // l3FrontRight = new Pose2d(
                                // new Translation2d(
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.INTAKE_X_OFFSET_FRONT,
                                // Physical.INTAKE_Y_OFFSET_FRONT,
                                // new Rotation2d(Math.PI)))
                                // .getX(),
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.INTAKE_X_OFFSET_FRONT,
                                // Physical.INTAKE_Y_OFFSET_FRONT,
                                // new Rotation2d(Math.PI)))
                                // .getY()),
                                // new Rotation2d(
                                // poseDirection.getRotation().getRadians() - Math.PI));
                                // l3BackRight = new Pose2d(
                                // new Translation2d(
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.INTAKE_X_OFFSET_BACK,
                                // Physical.INTAKE_Y_OFFSET_BACK,
                                // new Rotation2d()))
                                // .getX(),
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.INTAKE_X_OFFSET_BACK,
                                // Physical.INTAKE_Y_OFFSET_BACK,
                                // new Rotation2d()))
                                // .getY()),
                                // new Rotation2d(
                                // poseDirection.getRotation().getRadians()));
                                // l2FrontLeft = new Pose2d(
                                // new Translation2d(
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // -adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.L2_INTAKE_X_OFFSET_FRONT,
                                // Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                // new Rotation2d(Math.PI)))
                                // .getX(),
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // -adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.L2_INTAKE_X_OFFSET_FRONT,
                                // Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                // new Rotation2d(Math.PI)))
                                // .getY()),
                                // new Rotation2d(
                                // poseDirection.getRotation().getRadians() - Math.PI));
                                // l2BackLeft = new Pose2d(
                                // new Translation2d(
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // -adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.L2_INTAKE_X_OFFSET_BACK,
                                // Physical.L2_INTAKE_Y_OFFSET_BACK,
                                // new Rotation2d()))
                                // .getX(),
                                // poseDirection
                                // .transformBy(new Transform2d(adjustX,
                                // -adjustY,
                                // new Rotation2d()))
                                // .transformBy(new Transform2d(
                                // Physical.L2_INTAKE_X_OFFSET_BACK,
                                // Physical.L2_INTAKE_Y_OFFSET_BACK,
                                // new Rotation2d()))
                                // .getY()),
                                // new Rotation2d(
                                // poseDirection.getRotation().getRadians()));
                                frontLeftMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                if (poseDirection.getRotation().getDegrees() > 179.0
                                                && poseDirection.getRotation().getDegrees() < 181.0) {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + A_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + A_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("b branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > 119.0
                                                && poseDirection.getRotation().getDegrees() < 121.0) {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + K_BRANCH_OFFSET,
                                                                                                        -adjustY + K_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + K_BRANCH_OFFSET,
                                                                                                        -adjustY + K_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + K_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + K_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + K_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + K_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("d branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > 59.0
                                                && poseDirection.getRotation().getDegrees() < 61.0) {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + I_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + I_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("f branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > -1.0
                                                && poseDirection.getRotation().getDegrees() < 1.0) {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + G_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + G_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("h branch: " + l4FrontRight);
                                } else if (poseDirection.getRotation().getDegrees() > -61.0
                                                && poseDirection.getRotation().getDegrees() < -59.0) {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));
                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + E_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + E_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("j branch: " + l4FrontRight);
                                } else {
                                        l4FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l3FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET_L3,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE_L3,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        l2FrontLeft = new Pose2d(
                                                        new Translation2d(
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getX(),
                                                                        poseDirection
                                                                                        .transformBy(new Transform2d(
                                                                                                        adjustX + C_BRANCH_OFFSET_L2,
                                                                                                        -adjustY + C_BRANCH_OFFSET_SIDE_L2,
                                                                                                        new Rotation2d()))
                                                                                        .transformBy(new Transform2d(
                                                                                                        Physical.L2_INTAKE_X_OFFSET_FRONT,
                                                                                                        Physical.L2_INTAKE_Y_OFFSET_FRONT,
                                                                                                        new Rotation2d(Math.PI)))
                                                                                        .getY()),
                                                        new Rotation2d(
                                                                        poseDirection.getRotation().getRadians()
                                                                                        - Math.PI));

                                        System.out.println("l branch: " + l4FrontRight);
                                }

                                blueFrontPlacingPositions.add(l2FrontRight);
                                blueFrontPlacingPositions.add(l2FrontLeft);
                                blueFrontPlacingPositionsMore.add(frontRightMore);
                                blueFrontPlacingPositionsMore.add(frontLeftMore);
                                l4BlueFrontPlacingPositions.add(l4FrontRight);
                                l4BlueFrontPlacingPositions.add(l4FrontLeft);
                                l3BlueFrontPlacingPositions.add(l3FrontRight);
                                l3BlueFrontPlacingPositions.add(l3FrontLeft);
                                algaeBlueFrontPlacingPositions.add(algaeFront);
                                algaeBlueFrontPlacingPositionsMore.add(algaeFrontMore);
                                blueL1FrontPlacingPositions.add(l1FrontLeft);
                                blueL1FrontPlacingPositions.add(l1FrontRight);
                        }

                        for (Pose2d bluePose : blueL1FrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redL1FrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : algaeBlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : algaeBlueFrontPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedFrontPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : blueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : blueFrontPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redFrontPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : l4BlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l4RedFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : l3BlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l3RedFrontPlacingPositions.add(redPose);
                        }

                }
        }

        // Physical constants (e.g. field and robot dimensions)
        public static final class Physical {
                public static final double FIELD_WIDTH = 8.052;
                public static final double FIELD_LENGTH = 17.548;
                public static final double WHEEL_DIAMETER = inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
                public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;
                public static final double WHEEL_TO_FRAME_DISTANCE = inchesToMeters(2.5);
                public static final double TOP_SPEED = feetToMeters(30.0);

                public static final double ROBOT_LENGTH = inchesToMeters(26);
                public static final double ROBOT_WIDTH = inchesToMeters(26);
                public static final double MODULE_OFFSET = inchesToMeters(2.625);
                public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH / 2 - WHEEL_TO_FRAME_DISTANCE,
                                ROBOT_WIDTH / 2 - WHEEL_TO_FRAME_DISTANCE);
                public static double INTAKE_X_OFFSET_FRONT = inchesToMeters(23.8);
                public static double INTAKE_Y_OFFSET_FRONT = inchesToMeters(0.7);
                public static double INTAKE_X_OFFSET_BACK = inchesToMeters(23.8);
                public static double INTAKE_Y_OFFSET_BACK = inchesToMeters(-0.7);

                public static double INTAKE_X_OFFSET_FRONT_ALGAE = inchesToMeters(23.0 + 5.0);
                public static double INTAKE_Y_OFFSET_FRONT_ALGAE = inchesToMeters(3.8);
                public static double INTAKE_X_OFFSET_BACK_ALGAE = inchesToMeters(23.0 + 5.0);
                public static double INTAKE_Y_OFFSET_BACK_ALGAE = inchesToMeters(-3.8);

                public static double L1_INTAKE_X_OFFSET_FRONT = inchesToMeters(35.3);
                public static double L1_INTAKE_Y_OFFSET_FRONT = inchesToMeters(5.0);
                public static double L1_INTAKE_X_OFFSET_BACK = inchesToMeters(35.3);
                public static double L1_INTAKE_Y_OFFSET_BACK = inchesToMeters(-7.0);

                public static double L1_INTAKE_X_OFFSET_FRONT_MORE = inchesToMeters(24.5);
                public static double L1_INTAKE_Y_OFFSET_FRONT_MORE = inchesToMeters(5.0);
                public static double L1_INTAKE_X_OFFSET_BACK_MORE = inchesToMeters(24.5);
                public static double L1_INTAKE_Y_OFFSET_BACK_MORE = inchesToMeters(-9.0);

                public static double L2_INTAKE_X_OFFSET_FRONT = inchesToMeters(23.45);
                public static double L2_INTAKE_Y_OFFSET_FRONT = inchesToMeters(0.7);
                public static double L2_INTAKE_X_OFFSET_BACK = inchesToMeters(23.45);
                public static double L2_INTAKE_Y_OFFSET_BACK = inchesToMeters(-0.7);

                public static double L4_INTAKE_X_OFFSET_FRONT = inchesToMeters(27.1);
                public static double L4_INTAKE_Y_OFFSET_FRONT = inchesToMeters(0.7);
                public static double L4_INTAKE_X_OFFSET_BACK = inchesToMeters(27.1);
                public static double L4_INTAKE_Y_OFFSET_BACK = inchesToMeters(-0.7);

                public static final double GRAVITY_ACCEL_MS2 = 9.806;
        }

        // Subsystem setpoint constants
        public static final class SetPoints {
                public static class IntakeSetpoints {
                        public static final double INTAKE_ACCELERATION = 500.0;
                        public static final double INTAKE_CRUISE_VELOCITY = 400.0;
                        public static final double INTAKE_MOTION_PROFILE_SCALAR = 1.0;
                        public static final double INTAKE_DOWN = 0.36163; // rotations
                        public static final double INTAKE_UP = -0.014; // rotations
                        public static final double INTAKE_ROLLER_MAX_SPEED = 1.0; // percent
                        public static final double INTAKE_ROLLER_HOLDING_SPEED = 0.1; // percent
                        public static final double INTAKE_ROLLER_TORQUE = 80.0; // amps
                        public static final double INTAKE_HOLDING_TORQUE = 60.0; // amps
                }

                public static final double ELEVATOR_BOTTOM_POSITION_M = 0.0;
                public static final double ELEVATOR_MID_POSITION_M = inchesToMeters(26.0); // L2 after placement
                public static final double ELEVATOR_TOP_POSITION_M = inchesToMeters(43.0);
                public static final double ELEVATOR_L1_POSITION_M = inchesToMeters(6.6);
                public static final double ELEVATOR_L2_POSITION_M = inchesToMeters(15);
                public static final double ELEVATOR_AUTO_L2_POSITION_M = inchesToMeters(20);
                public static final double ELEVATOR_AUTO_L2_POSITION_SCORE_M = inchesToMeters(16);
                public static final double ELEVATOR_AUTO_L3_POSITION_M = inchesToMeters(35.75);
                // public static final double ELEVATOR_AUTO_L3_POSITION_M = inchesToMeters(25);
                public static final double ELEVATOR_AUTO_SCORE_L3_POSITION_M = inchesToMeters(20);
                public static final double ELEVATOR_AUTO_L4_POSITION_M = inchesToMeters(64.0);
                public static final double ELEVATOR_L3_POSITION_M = inchesToMeters(28);
                public static final double ELEVATOR_L4_POSITION_M = inchesToMeters(64.0);
                public static final double ELEVATOR_ALGAE_POSITION_M = inchesToMeters(8.0);
                public static final double ELEVATOR_GROUND_CORAL_POSITION_M = inchesToMeters(5.4);
                public static final double ELEVATOR_GROUND_ALGAE_POSITION_M = inchesToMeters(0.0);
                public static final double ELEVATOR_FEEDER_POSITION_M = inchesToMeters(0.0);
                public static final double ELEVATOR_OVER_POSITION_M = inchesToMeters(20);
                public static final double ELEVATOR_NET_POSITION_M = inchesToMeters(65);
                public static final double ELEVATOR_L2_ALGAE_POSITION_M = inchesToMeters(15.7);
                public static final double ELEVATOR_L3_ALGAE_POSITION_M = inchesToMeters(33.0);
                public static final double ELEVATOR_PROCESSOR_POSITION_M = inchesToMeters(6.5);
                public static final double ELEVATOR_LOLLIPOP_POSITION_M = inchesToMeters(0.0);
                public static final double ELEVATOR_PRE_HANDOFF_POSITION_M = inchesToMeters(39.0);
                public static final double ELEVATOR_HANDOFF_POSITION_M = inchesToMeters(35.0);

                public enum ElevatorPosition {
                        kDOWN(ELEVATOR_BOTTOM_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_BOTTOM_POSITION_M)),
                        kMID(ELEVATOR_MID_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_MID_POSITION_M)),
                        kUP(ELEVATOR_TOP_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_TOP_POSITION_M)),
                        kL1(ELEVATOR_L1_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L1_POSITION_M)),
                        kL2(ELEVATOR_L2_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L2_POSITION_M)),
                        kAUTOL2(ELEVATOR_AUTO_L2_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L2_POSITION_M)),
                        kAUTOL2SCORE(ELEVATOR_AUTO_L2_POSITION_SCORE_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L2_POSITION_SCORE_M)),
                        kAUTOL3(ELEVATOR_AUTO_L3_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L3_POSITION_M)),
                        kAUTOL3SCORE(ELEVATOR_AUTO_SCORE_L3_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_SCORE_L3_POSITION_M)),
                        kAUTOL4(ELEVATOR_AUTO_L4_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L4_POSITION_M)),
                        kL3(ELEVATOR_L3_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L3_POSITION_M)),
                        kL4(ELEVATOR_L4_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L4_POSITION_M)),
                        kALGAE(ELEVATOR_ALGAE_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_ALGAE_POSITION_M)),
                        kFEEDER(ELEVATOR_FEEDER_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_FEEDER_POSITION_M)),
                        kGROUNDCORAL(ELEVATOR_GROUND_CORAL_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_GROUND_CORAL_POSITION_M)),
                        kGROUNDALGAE(ELEVATOR_GROUND_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_GROUND_ALGAE_POSITION_M)),
                        kNET(ELEVATOR_NET_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_NET_POSITION_M)),
                        kPROCESSOR(ELEVATOR_PROCESSOR_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_PROCESSOR_POSITION_M)),
                        kL2ALGAE(ELEVATOR_L2_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_L2_ALGAE_POSITION_M)),
                        kL3ALGAE(ELEVATOR_L3_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_L3_ALGAE_POSITION_M)),
                        kOVER(ELEVATOR_OVER_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_OVER_POSITION_M)),
                        kLOLLIPOP(ELEVATOR_LOLLIPOP_POSITION_M, Ratios.elevatorMetersToRotations(
                                        ELEVATOR_LOLLIPOP_POSITION_M)),
                        kHANDOFF(ELEVATOR_HANDOFF_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_HANDOFF_POSITION_M)),
                        kPREHANDOFF(ELEVATOR_PRE_HANDOFF_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_PRE_HANDOFF_POSITION_M));

                        public final double meters;
                        public final double rotations;

                        private ElevatorPosition(double meters, double rotations) {
                                this.meters = meters;
                                this.rotations = rotations;
                        }
                }

                public static final double PIVOT_L1_POSITION_D = 67.0;
                public static final double PIVOT_L23_POSITION_D = 52.5;
                // public static final double PIVOT_AUTO_L23_POSITION_D = 45.0;
                public static final double PIVOT_AUTO_L2_POSITION_D = 65.0;
                public static final double PIVOT_AUTO_L3_POSITION_D = 50.0;
                // public static final double PIVOT_AUTO_L3_POSITION_D = 30.0;
                public static final double PIVOT_AUTO_L4_POSITION_D = 0.0;
                public static final double PIVOT_AUTO_L4_SCORE_POSITION_D = 100.0;
                public static final double PIVOT_AUTO_L4_SCORE_SLOW_POSITION_D = 70.0;
                public static final double PIVOT_AUTO_L3_SCORE_POSITION_D = 100.0;
                public static final double PIVOT_AUTO_L2_SCORE_POSITION_D = 100.0;
                public static final double PIVOT_L4_POSITION_D = 60.0;
                public static final double PIVOT_UPRIGHT_POSITION_D = 45.0;
                public static final double PIVOT_GROUND_ALGAE_POSITION_D = 82.5;
                public static final double PIVOT_GROUND_CORAL_POSITION_FRONT_D = 111.0;
                public static final double PIVOT_GROUND_CORAL_POSITION_BACK_D = -111.0;
                public static final double PIVOT_GROUND_CORAL_PREP_BACK_D = -90;
                // public static final double PIVOT_DEFAULT_POSITION_D = 30.0;
                public static final double PIVOT_DEFAULT_POSITION_D = 0.0;
                public static final double PIVOT_DEFAULT_CLIMB_POSITION_D = 45.0;
                public static final double PIVOT_PREP_POSITION_D = 30.0;
                public static final double PIVOT_FEEDER_POSITION_D = 21.0;
                public static final double PIVOT_NET_POSITION_D = 15.0;
                public static final double PIVOT_PROCESSOR_POSITION_D = 76.0;
                public static final double PIVOT_REEF_ALGAE_POSITION_D = 80.0;
                public static final double PIVOT_CLIMB_POSITION_D = 45.0;
                public static final double PIVOT_LOLLIPOP_POSITION_D = -98.0;
                public static final double PIVOT_HANDOFF_POSITION_D = 145.0;

                public enum PivotPosition {
                        kL1(PIVOT_L1_POSITION_D, Constants.degreesToRotations(PIVOT_L1_POSITION_D)),
                        kL23(PIVOT_L23_POSITION_D, Constants.degreesToRotations(PIVOT_L23_POSITION_D)),
                        kAUTOL2(PIVOT_AUTO_L2_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L2_POSITION_D)),
                        kAUTOL3(PIVOT_AUTO_L3_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L3_POSITION_D)),
                        kAUTOL4(PIVOT_AUTO_L4_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L4_POSITION_D)),
                        kL4(PIVOT_L4_POSITION_D, Constants.degreesToRotations(PIVOT_L4_POSITION_D)),
                        kUP(PIVOT_UPRIGHT_POSITION_D, Constants.degreesToRotations(PIVOT_UPRIGHT_POSITION_D)),
                        kGROUNDALGAE(PIVOT_GROUND_ALGAE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_ALGAE_POSITION_D)),
                        kGROUNDCORALFRONT(PIVOT_GROUND_CORAL_POSITION_FRONT_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_POSITION_FRONT_D)),
                        kGROUNDCORALBACK(PIVOT_GROUND_CORAL_POSITION_BACK_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_POSITION_BACK_D)),
                        kGROUNDCORALPREPBACK(PIVOT_GROUND_CORAL_PREP_BACK_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_PREP_BACK_D)),
                        kNET(PIVOT_NET_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_NET_POSITION_D)),
                        kPROCESSOR(PIVOT_PROCESSOR_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_PROCESSOR_POSITION_D)),
                        kAUTOL2SCORE(PIVOT_AUTO_L2_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L2_SCORE_POSITION_D)),
                        kAUTOL3SCORE(PIVOT_AUTO_L3_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L3_SCORE_POSITION_D)),
                        kAUTOL4SCORE(PIVOT_AUTO_L4_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L4_SCORE_POSITION_D)),
                        kAUTOL4SCORESLOW(PIVOT_AUTO_L4_SCORE_SLOW_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L4_SCORE_SLOW_POSITION_D)),
                        kDEFAULT(PIVOT_DEFAULT_POSITION_D, Constants.degreesToRotations(PIVOT_DEFAULT_POSITION_D)),
                        kDEFAULTCLIMB(PIVOT_DEFAULT_CLIMB_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_DEFAULT_CLIMB_POSITION_D)),
                        kFEEDER(PIVOT_FEEDER_POSITION_D, Constants.degreesToRotations(PIVOT_FEEDER_POSITION_D)),
                        kREEFALGAE(PIVOT_REEF_ALGAE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_REEF_ALGAE_POSITION_D)),
                        kPREP(PIVOT_PREP_POSITION_D, Constants.degreesToRotations(PIVOT_PREP_POSITION_D)),
                        kCLIMB(PIVOT_CLIMB_POSITION_D, Constants.degreesToRotations(PIVOT_CLIMB_POSITION_D)),
                        kLOLLIPOP(PIVOT_LOLLIPOP_POSITION_D, Constants.degreesToRotations(PIVOT_LOLLIPOP_POSITION_D)),
                        kHANDOFF(PIVOT_HANDOFF_POSITION_D, Constants.degreesToRotations(PIVOT_HANDOFF_POSITION_D));

                        public final double degrees;
                        public final double rotations;

                        private PivotPosition(double degrees, double rotations) {
                                this.degrees = degrees;
                                this.rotations = rotations;
                        }
                }
        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {
                // Poses of all 16 AprilTags, {x, y, z, yaw, pitch}, in meters and radians
                public static final double[][] TAG_POSES = {
                                { inchesToMeters(657.37), inchesToMeters(25.8), inchesToMeters(58.5),
                                                Math.toRadians(126),
                                                Math.toRadians(0) },
                                { inchesToMeters(657.37), inchesToMeters(291.2), inchesToMeters(58.5),
                                                Math.toRadians(234),
                                                Math.toRadians(0) },
                                { inchesToMeters(455.15), inchesToMeters(317.15), inchesToMeters(51.25),
                                                Math.toRadians(270),
                                                Math.toRadians(0) },
                                { inchesToMeters(365.2), inchesToMeters(241.64), inchesToMeters(73.54),
                                                Math.toRadians(0),
                                                Math.toRadians(30) },
                                { inchesToMeters(365.2), inchesToMeters(75.39), inchesToMeters(73.54),
                                                Math.toRadians(0),
                                                Math.toRadians(30) },
                                { inchesToMeters(530.49), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(300),
                                                Math.toRadians(0) },
                                { inchesToMeters(546.87), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(0),
                                                Math.toRadians(0) },
                                { inchesToMeters(530.49), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(60),
                                                Math.toRadians(0) },
                                { inchesToMeters(497.77), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(120),
                                                Math.toRadians(0) },
                                { inchesToMeters(481.39), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(180),
                                                Math.toRadians(0) },
                                { inchesToMeters(497.77), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(240),
                                                Math.toRadians(0) },
                                { inchesToMeters(33.51), inchesToMeters(25.8), inchesToMeters(58.5), Math.toRadians(54),
                                                Math.toRadians(0) },
                                { inchesToMeters(33.51), inchesToMeters(291.2), inchesToMeters(58.5),
                                                Math.toRadians(306),
                                                Math.toRadians(0) },
                                { inchesToMeters(325.68), inchesToMeters(241.64), inchesToMeters(73.54),
                                                Math.toRadians(180),
                                                Math.toRadians(30) },
                                { inchesToMeters(325.68), inchesToMeters(75.39), inchesToMeters(73.54),
                                                Math.toRadians(180),
                                                Math.toRadians(30) },
                                { inchesToMeters(235.73), inchesToMeters(-0.15), inchesToMeters(51.25),
                                                Math.toRadians(90),
                                                Math.toRadians(0) },
                                { inchesToMeters(160.39), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(240),
                                                Math.toRadians(0) },
                                { inchesToMeters(144), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(180),
                                                Math.toRadians(0) },
                                { inchesToMeters(160.39), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(120),
                                                Math.toRadians(0) },
                                { inchesToMeters(193.1), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(60),
                                                Math.toRadians(0) },
                                { inchesToMeters(209.49), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(0),
                                                Math.toRadians(0) },
                                { inchesToMeters(193.1), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(300),
                                                Math.toRadians(0) }
                };

                public static final double[][] redSideReefTags = {
                                TAG_POSES[6 - 1], TAG_POSES[7 - 1], TAG_POSES[8 - 1], TAG_POSES[9 - 1],
                                TAG_POSES[10 - 1],
                                TAG_POSES[11 - 1] };

                public static final double[][] blueSideReefTags = {
                                TAG_POSES[6 + 11 - 1], TAG_POSES[7 + 11 - 1], TAG_POSES[8 + 11 - 1],
                                TAG_POSES[9 + 11 - 1],
                                TAG_POSES[10 + 11 - 1], TAG_POSES[11
                                                + 11 - 1] };

                // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
                // radians
                public static final double[] FRONT_CAMERA_POSE = { Constants.inchesToMeters(1.75),
                                Constants.inchesToMeters(11.625),
                                Constants.inchesToMeters(33.5), 0, -33.5, 0 };

                // Standard deviation adjustments
                public static final double STANDARD_DEVIATION_SCALAR = 1;
                public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR = 1;
                public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE = 3;
                public static final double TAG_STANDARD_DEVIATION_DISTANCE = 2; // meters
                public static final double TAG_STANDARD_DEVIATION_FLATNESS = 5;
                // public static final double XY_STD_DEV_SCALAR = 0.1;

                // Standard deviation regressions
                /**
                 * Calculates the standard deviation scalar based on the distance from the tag.
                 *
                 * @param dist The distance from the tag.
                 * @return The standard deviation scalar.
                 */
                public static double getTagDistStdDevScalar(double dist) {
                        // double a = TAG_STANDARD_DEVIATION_FLATNESS;
                        // double b = 1 - a * Math.pow(TAG_STANDARD_DEVIATION_DISTANCE, 2);
                        // Logger.recordOutput("std devs",
                        // -0.000277778 * Math.pow(dist, 3) + 0.00988095 * Math.pow(dist, 2) +
                        // 0.00444444 * dist + 0.0371429);
                        // return Math.max(1, a * Math.pow(dist, 2) + b);
                        return -0.00045928 * Math.pow(dist, 4) + 0.0069476 * Math.pow(dist, 3)
                                        - 0.0216241 * Math.pow(dist, 2)
                                        + 0.063534 * dist + 0.0317614;
                }

                /**
                 * Calculates the standard deviation scalar based on the number of detected
                 * tags.
                 *
                 * @param numTags The number of detected tags.
                 * @return The standard deviation scalar.
                 */
                public static double getNumTagStdDevScalar(int numTags) {
                        if (numTags == 0) {
                                return 99999;
                        } else if (numTags == 1) {
                                return 1;
                        } else if (numTags == 2) {
                                return 0.6;
                        } else {
                                return 0.75;
                        }
                }

                /**
                 * Calculates the standard deviation of the x-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the x-coordinate.
                 */
                public static double getTagStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.010807566510145635)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation of the y-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the y-coordinate.
                 */
                public static double getTagStdDevY(double xOffset, double yOffset) {
                        return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the x-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the x-coordinate.
                 */
                public static double getTriStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.01955724864971872)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the y-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the y-coordinate.
                 */
                public static double getTriStdDevY(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.002615358015002413 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.008955462032388808)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                public static double distBetweenPose(Pose3d pose1, Pose3d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }

                public static double distBetweenPose2d(Pose2d pose1, Pose2d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }

                public static final double DISTANCE_OFFSET = 7.0;
                public static final double CAMERA_ANGLE_OFFSET = 0.0;
                // pitch, distance
                public static final double[][] CORAL_LOOKUP_TABLE = {
                                { -17.72 + CAMERA_ANGLE_OFFSET, 24.5 + DISTANCE_OFFSET },
                                { -12.22 + CAMERA_ANGLE_OFFSET, 28.5 + DISTANCE_OFFSET },
                                { -8.02 + CAMERA_ANGLE_OFFSET, 33.0 + DISTANCE_OFFSET },
                                { -5.69 + CAMERA_ANGLE_OFFSET, 38.25 + DISTANCE_OFFSET },
                                { -2.73 + CAMERA_ANGLE_OFFSET, 43.75 + DISTANCE_OFFSET },
                                { -0.05 + CAMERA_ANGLE_OFFSET, 50.0 + DISTANCE_OFFSET },
                                { 2.09 + CAMERA_ANGLE_OFFSET, 56.0 + DISTANCE_OFFSET },
                                { 3.55 + CAMERA_ANGLE_OFFSET, 61.5 + DISTANCE_OFFSET },
                                { 5.87 + CAMERA_ANGLE_OFFSET, 71.75 + DISTANCE_OFFSET }
                };

                /**
                 * Interpolates a value from a lookup table based on the given xValue.
                 * 
                 * @param xIndex The index of the x-values in the lookup table.
                 * @param yIndex The index of the y-values in the lookup table.
                 * @param xValue The x-value for which to interpolate a y-value.
                 * @return The interpolated y-value corresponding to the given x-value.
                 */
                public static double getInterpolatedValue(int xIndex, int yIndex, double xValue) {
                        int lastIndex = CORAL_LOOKUP_TABLE.length - 1;
                        if (xValue < CORAL_LOOKUP_TABLE[0][xIndex]) {
                                // If the xValue is closer than the first setpoint
                                double returnValue = CORAL_LOOKUP_TABLE[0][yIndex];
                                return returnValue;
                        } else if (xValue > CORAL_LOOKUP_TABLE[lastIndex][xIndex]) {
                                // If the xValue is farther than the last setpoint
                                double returnValue = CORAL_LOOKUP_TABLE[lastIndex][yIndex];
                                return returnValue;
                        } else {
                                for (int i = 0; i < CORAL_LOOKUP_TABLE.length; i++) {
                                        if (xValue > CORAL_LOOKUP_TABLE[i][xIndex]
                                                        && xValue < CORAL_LOOKUP_TABLE[i + 1][xIndex]) {
                                                // If the xValue is in the table of setpoints
                                                // Calculate where xValue is between setpoints
                                                double leftDif = xValue - CORAL_LOOKUP_TABLE[i][xIndex];
                                                double percent = leftDif / (CORAL_LOOKUP_TABLE[i + 1][xIndex]
                                                                - CORAL_LOOKUP_TABLE[i][xIndex]);

                                                double value1 = CORAL_LOOKUP_TABLE[i][yIndex];
                                                double value2 = CORAL_LOOKUP_TABLE[i + 1][yIndex];

                                                // Interpolate in-between values for value xValue and shooter rpm
                                                double newValue = value1 + (percent * (value2 - value1));

                                                double returnValue = newValue;
                                                return returnValue;
                                        }
                                }
                                // Should never run
                                double returnValue = CORAL_LOOKUP_TABLE[0][yIndex];
                                return returnValue;
                        }
                }

                /**
                 * Calculates shooter values (flywheel velocity and note velocity) based on the
                 * given angle.
                 *
                 * @param angle The angle of the shooter.
                 * @return An array containing the calculated flywheel velocity and note
                 *         velocity.
                 */
                public static double getCoralDistanceFromPitch(double angle) {
                        return getInterpolatedValue(0, 1, angle);
                        // return new double[] {25, getInterpolatedValue(1, 3, angle)};
                }
        }

        // Gear ratios and conversions
        public static final class Ratios {

                // pivot
                public static final double PIVOT_GEAR_RATIO = 23 * 64 / 24;

                // drive
                public static final double DRIVE_GEAR_RATIO = 6.12;
                public static final double STEER_GEAR_RATIO = 21.43;
                public static final double MAX_WHEEL_RPS = 6380.0 / 60.0 / 6.12;

                // elevator
                public static final double ELEVATOR_FIRST_STAGE = Constants.inchesToMeters(20);
                public static final double ELEVATOR_MOTOR_ROTATIONS_FOR_FIRST_STAGE = 18.067;
                public static final double ELEVATOR_MOTOR_ROTATIONS_PER_METER = ELEVATOR_MOTOR_ROTATIONS_FOR_FIRST_STAGE
                                * (1 / ELEVATOR_FIRST_STAGE);

                public static double elevatorRotationsToMeters(double rotations) {
                        return rotations / ELEVATOR_MOTOR_ROTATIONS_PER_METER;
                }

                public static double elevatorMetersToRotations(double meters) {
                        return meters * ELEVATOR_MOTOR_ROTATIONS_PER_METER;
                }

                // intake
                public static final double INTAKE_PIVOT_GEAR_RATIO = 50.0;
        }

        // Can info such as IDs
        public static final class CANInfo {
                public static final String CANBUS_NAME = "Canivore";

                // drive
                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
                public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 2;
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
                public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 4;
                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
                public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
                public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 8;
                public static final int FRONT_RIGHT_MODULE_CANCODER_ID = 1;
                public static final int FRONT_LEFT_MODULE_CANCODER_ID = 2;
                public static final int BACK_LEFT_MODULE_CANCODER_ID = 3;
                public static final int BACK_RIGHT_MODULE_CANCODER_ID = 4;

                // Lights
                public static final int CANDLE_ID_0 = 0;
                public static final int CANDLE_ID_1 = 1;
                public static final int CANDLE_ID_2 = 2;

                // Elevator
                public static final int LEFT_ELEVATOR_MOTOR_ID = 9;
                public static final int RIGHT_ELEVATOR_MOTOR_ID = 10;

                // Pivot
                public static final int PIVOT_MOTOR_ID = 11;
                public static final int PIVOT_CANCODER_ID = 5;

                // Twist
                public static final int TWIST_MOTOR_ID = 12;
                public static final int TWIST_CANCODER_ID = 6;

                // Manipulator

                public static final int MANIPULATOR_MOTOR_ID = 13;

                // Climber
                public static final int CLIMBER_PIVOT_MOTOR_ID = 15;

                // Intake
                public static final int INTAKE_ROLLER_MOTOR_ID = 10;
                public static final int INTAKE_PIVOT_MOTOR_ID = 11;
                public static final int INTAKE_BEAM_BREAK_PORT = 0;

                // Straightenator
                public static final int LEFT_STRAIGHTENATOR_MOTOR_ID = 12;
                public static final int RIGHT_STRAIGHTENATOR_MOTOR_ID = 13;
                public static final int FAR_BEAM_BREAK_SENSOR = 0;
                public static final int CLOSE_BEAM_BREAK_SENSOR = 2;

                // Arm
                public static final int ARM_PIVOT_MOTOR_ID = 15;
                public static final int ARM_MANIPULATOR_MOTOR_ID = 14;
        }

        public static final class IntakeConstants {
                public static double PIVOT_LOWER_LIMIT = 0;
                public static double PIVOT_UPPER_LIMIT = 10;
        }

        // Misc. controller values
        public static final class OperatorConstants {
                public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_STICK_DEADZONE = 0.03;
                public static final double RIGHT_STICK_DEADZONE = 0.05;
        }

        /**
         * Converts inches to meters.
         *
         * @param inches The length in inches to be converted.
         * @return The equivalent length in meters.
         */
        public static double inchesToMeters(double inches) {
                return inches / 39.37;
        }

        public static double metersToInches(double meters) {
                return meters * 39.37;
        }

        /**
         * Converts feet to meters.
         *
         * @param inches The length in feet to be converted.
         * @return The equivalent length in meters.
         */
        public static double feetToMeters(double feet) {
                return feet / 3.281;
        }

        /**
         * Calculates the Euclidean distance between two points in a 2D plane.
         *
         * @param x1 The x-coordinate of the first point.
         * @param y1 The y-coordinate of the first point.
         * @param x2 The x-coordinate of the second point.
         * @param y2 The y-coordinate of the second point.
         * @return The Euclidean distance between the two points.
         */
        public static double getDistance(double x1, double y1, double x2, double y2) {
                return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        }

        public static double getAngleToPoint(double x1, double y1, double x2, double y2) {
                // System.out.println("x1: " + x1 + ", y1: " + y1 + ", x2: " + x2 + ", y2: " +
                // y2);
                double deltaX = x2 - x1;
                double deltaY = y2 - y1;

                double angleInRadians = Math.atan2(deltaY, deltaX);

                double angleInDegrees = Math.toDegrees(angleInRadians);

                double standardizeAngleDegrees = standardizeAngleDegrees(angleInDegrees);

                // if (y1 > y2) {
                // System.out.println("running");
                return 180 + standardizeAngleDegrees;
                // }
                // System.out.println("2");
                // double temp = 180 - standardizeAngleDegrees;
                // double j = 180 - temp;
                // return 180 + j;
                // }
        }

        /**
         * Converts a quantity in rotations to radians.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double rotationsToRadians(double rotations) {
                return rotations * 2 * Math.PI;
        }

        /**
         * Converts a quantity in degrees to rotations.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in rotations.
         */
        public static double degreesToRotations(double degrees) {
                return degrees / 360;
        }

        /**
         * Converts a quantity in rotations to degrees.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in degrees.
         */
        public static double rotationsToDegrees(double rotations) {
                return rotations * 360;
        }

        /**
         * Converts a quantity in degrees to radians.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double degreesToRadians(double degrees) {
                return degrees * Math.PI / 180;
        }

        /**
         * Standardizes an angle to be within the range [0, 360) degrees.
         *
         * @param angleDegrees The input angle in degrees.
         * @return The standardized angle within the range [0, 360) degrees.
         */
        public static double standardizeAngleDegrees(double angleDegrees) {
                // System.out.println("initial angle degrees" + angleDegrees);
                if (angleDegrees >= 0 && angleDegrees < 360) {
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else if (angleDegrees < 0) {
                        while (angleDegrees < 0) {
                                angleDegrees += 360;
                        }
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else if (angleDegrees >= 360) {
                        while (angleDegrees >= 360) {
                                angleDegrees -= 360;
                        }
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else {
                        // System.out.println("Weird ErroR");
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                }
        }

        /**
         * Calculates the x-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The x-component of the unit vector.
         */
        public static double angleToUnitVectorI(double angle) {
                return (Math.cos(angle));
        }

        /**
         * Calculates the y-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The y-component of the unit vector.
         */
        public static double angleToUnitVectorJ(double angle) {
                return (Math.sin(angle));
        }

        /**
         * Converts revolutions per minute (RPM) to revolutions per second (RPS).
         *
         * @param RPM The value in revolutions per minute (RPM) to be converted.
         * @return The equivalent value in revolutions per second (RPS).
         */
        public static double RPMToRPS(double RPM) {
                return RPM / 60;
        }

        /**
         * Converts revolutions per second (RPS) to revolutions per minute (RPM).
         *
         * @param RPM The value in revolutions per second (RPS) to be converted.
         * @return The equivalent value in revolutions per minute (RPM).
         */
        public static double RPSToRPM(double RPS) {
                return RPS * 60;
        }
}