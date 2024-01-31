// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double maxVitesseLineaire = 1.5;//Vitesse linéaire max du chassis //Valeur original 4.8
    public static final double maxVitesseRotation = Math.PI; // radians per second //Originale REV = 2pi

    public static final double maxVitesseModule = 3;//Vitesse maximale d'un module en m/s 

    public static final double kDirectionSlewRate = 7; // radians per second //REV = 1.2
    public static final double kMagnitudeSlewRate = 2.25; // percent per second (1 = 100%) // REV 1.8
    public static final double kRotationalSlewRate = 2.25; // percent per second (1 = 100%) //REV 2.0

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    
    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
                                          new PIDConstants(0.5, 0.0, 0.0),
                                          new PIDConstants(5.0, 0.0, 0.0),
                                          maxVitesseModule,
                                          new Translation2d(kWheelBase / 2, kTrackWidth / 2).getNorm(),
                                          new ReplanningConfig());
  }

  public static final class ModuleConstants {

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;//vitesse libre = 5676 RPM
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(3) * Math.PI;// roue de 3 pouces
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (13 * 15);//pinion choisi 13 dents
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
 

    public static final double kTurningP = 1;
    public static final double kTurningFF = 0;
  

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

 


}
