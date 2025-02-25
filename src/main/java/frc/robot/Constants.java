// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    //* Swerve Constants */ //* Updated From 2024 Swerve Constants */
    public static final class Swerve {
         public static final double STEER_kP = 3.0;
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;
    public static final double STEER_RAMP_RATE = 0.15; // how many seconds to go from 0 to full throttle
    public static final int STEER_CURRENT_LIMIT = 20;

    public static final double DRIVE_kP = 0.05;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 0;
    public static final double DRIVE_kS = 0.24;
    public static final double DRIVE_kV = 0.115;
    public static final double DRIVE_RAMP_RATE = 0.15;

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = 
        new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);
    public static final int DRIVE_CURRENT_LIMIT = 40;
   
    public static final double MAX_OUTPUT = 0.3;

    public static final boolean SQUARED_INPUTS = true; // at 2024 SVR, this was false

    public static final double WHEEL_RADIUS = 2.0; // inches, need to double check
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public static final double DRIVE_POSITION_CONVERSION = 
        2.0 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS) / DRIVE_GEAR_RATIO; // meters per rotation
    public static final double DRIVE_VELOCITY_CONVERSION = 
        DRIVE_POSITION_CONVERSION / 60.0; // meters per second
    public static final double STEER_POSITION_CONVERSION = 1 / STEER_GEAR_RATIO; // rotations

    public static final int BR_DRIVE_ID = 1;
    public static final int FR_DRIVE_ID = 2;
    public static final int BL_DRIVE_ID = 3;
    public static final int FL_DRIVE_ID = 4;

    public static final int BR_STEER_ID = 5;
    public static final int FR_STEER_ID = 6;
    public static final int BL_STEER_ID = 7;
    public static final int FL_STEER_ID = 8;

    public static final int BR_ENCODER_ID = 9;
    public static final int FR_ENCODER_ID = 10;
    public static final int BL_ENCODER_ID = 11;
    public static final int FL_ENCODER_ID = 12;

    public static final int PIGEON_ID = 13;

    public static final boolean FR_DRIVE_INVERTED = true;
    public static final boolean FL_DRIVE_INVERTED = false;
    public static final boolean BR_DRIVE_INVERTED = true;
    public static final boolean BL_DRIVE_INVERTED = true;

    public static final boolean FR_STEER_INVERTED = true;
    public static final boolean FL_STEER_INVERTED = true;
    public static final boolean BR_STEER_INVERTED = true;
    public static final boolean BL_STEER_INVERTED = true;

    public static final double FR_OFFSET_ROTATIONS = 0.1644;
    public static final double FL_OFFSET_ROTATIONS = 0.5406;
    public static final double BR_OFFSET_ROTATIONS = 0.9320;
    public static final double BL_OFFSET_ROTATIONS = 0.2593;
    
    public static final double APOTHEM_1 = Units.inchesToMeters(0);
    public static final double APOTHEM_2 = Units.inchesToMeters(0); // NEED TO UPDATE FOR 2025
    public static final Translation2d FR_LOCATION = new Translation2d(APOTHEM_1, -APOTHEM_2);
    public static final Translation2d FL_LOCATION = new Translation2d(APOTHEM_1, APOTHEM_2);
    public static final Translation2d BR_LOCATION = new Translation2d(-APOTHEM_1, -APOTHEM_2);
    public static final Translation2d BL_LOCATION = new Translation2d(-APOTHEM_1, APOTHEM_2);
    }

}