// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.</p>
 */
public final class Constants {

public static final double topSpeedX = 1;
public static final double topSpeedY = 1;
public static final double topSpeedTurning = Math.PI / 4;

public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
public static final double kDriveMotorGearRatio = 1 / 5.8462;
public static final double kTurningMotorGearRatio = 1 / 18.0;
public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

//Settings for the Turning PID controller

public static final double TurningProportionalGain = 0.35;
  //Proportioal gain is % it is how fast the turning responds
  //Incressing the % leads to faster less stable loops
  //Decressing does the oppisite
public static double TurningReset = 1/15; // How fast the steedy state error is removed
public static double RateOfChange = 0; //We don't need the dirivitave part the D stands for disaster

public static final double DrivingProportionalGain = 0.015;
  //Proportioal gain is % it is how fast the turning responds
  //Incressing the % leads to faster less stable loops
  //Decressing does the oppisite
public static double DrivingReset = 1/15; // How fast the steedy state error is removed
public static double DrivingRateOfChange = 0; //We don't need the dirivitave part the D stands for disaster

public static final double AngleProportionalGain = 0.04;
  //Proportioal gain is % it is how fast the turning responds
  //Incressing the % leads to faster less stable loops
  //Decressing does the oppisite
public static double AngleReset = 1/15; // How fast the steedy state error is removed
public static double AngleRateOfChange = 0; //We don't need the dirivitave part the D stands for disaster

public static final double kTrackWidth = Units.inchesToMeters(29);
// Distance between right and left wheels
public static final double kWheelBase = Units.inchesToMeters(29);
// Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


public static double kPhysicalMaxSpeedMeterPerSecond = 15; 
public static double PhysicalMaxAcclerationUnitsPerSecond = 2;
public static double kDeadband = 0.001; //To be changed
public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;
}
