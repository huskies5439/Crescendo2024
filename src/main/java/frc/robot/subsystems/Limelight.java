// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable limelight = networkTableInstance.getTable("limelight");
  private NetworkTableEntry tv = limelight.getEntry("tv");
  private NetworkTableEntry ta = limelight.getEntry("ta");
  private NetworkTableEntry tl = limelight.getEntry("tl"); // Target Latency
  private NetworkTableEntry cl = limelight.getEntry("cl"); // Captured Latency

  private NetworkTableEntry botpose;

  double[] arrayLimelight;

  String alliance;

  public Limelight() {
    getAlliance();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Vision X", getVisionPosition().getX());
    // SmartDashboard.putNumber("Vision Y", getVisionPosition().getY());
    // SmartDashboard.putNumber("Vision Rotation ",
    // getVisionPosition().getRotation().getDegrees());
    // SmartDashboard.putNumber("Vision Total Latency", getTotalLatency());

  }

  // Donne la position du robot selon la limelight
  public Pose2d getVisionPosition() {
    arrayLimelight = botpose.getDoubleArray(new double[6]);

    Translation3d tran3d = new Translation3d(arrayLimelight[0], arrayLimelight[1], arrayLimelight[2]);
    Rotation3d r3d = new Rotation3d(Math.toRadians(arrayLimelight[3]), Math.toRadians(arrayLimelight[4]),
        Math.toRadians(arrayLimelight[5]));
    Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d.toPose2d();

  }

  // Temps d'attente avant la reception de l'image
  public double getTotalLatency() {
    return tl.getDouble(0) + cl.getDouble(0);
  }

  // Voit un April Tag
  public boolean getTv() {
    return tv.getDouble(0) == 1;
  }

  // Pourcentage que rempli le April Tag sur la vision
  public double getTa() {
    return ta.getDouble(0);
  }

  // Change l'alliance
  public void getAlliance() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        alliance = "red";
      }
      if (ally.get() == Alliance.Blue) {
        alliance = "blue";
      }
    } else {
      alliance = "blue";
    }

     botpose = limelight.getEntry("botpose_wpi" + alliance);

  }
}