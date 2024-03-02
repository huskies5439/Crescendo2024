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

public final class Constants {

  //////Les chiffres magiques sont les valeurs arbitraires qui se trouvent à plus d'un endroit dans le code
  public static final class ChiffreMagique{
    public static final double vitesseLancerSpeaker = 60;

    public static final double maxEchelle = 0.28;
    public static final double HauteurAmpli = 0.2;
  }
  

  ////////BasePilotable
  public static final class BPConstantes {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double maxVitesseLineaire = 4;// Vitesse linéaire max du chassis //Valeur original 4.8 a note:à
                                                      // ajuster
    public static final double maxVitesseRotation = Math.PI * 1.4; // radians per second //Originale REV = 2pi soirée 22
                                                                   // fevrier = 1.5

    public static final double maxVitesseModule = 4;// Vitesse maximale d'un module en m/s

    public static final double kDirectionSlewRate = 3; // radians per second
    public static final double kMagnitudeSlewRate = 2.5; // percent per second (1 = 100%) original : 1.8
    public static final double kRotationalSlewRate = 3; // percent per second (1 = 100%) original : 2

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

    // Pathplanner
    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(1.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        maxVitesseModule,
        new Translation2d(kWheelBase / 2, kTrackWidth / 2).getNorm(),
        new ReplanningConfig());
  }

}
