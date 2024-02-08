// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gobeur extends SubsystemBase {

  private final TalonFX moteur = new TalonFX(5);

  /** Creates a new Gobeur. */
  public Gobeur() {
    moteur.setInverted(false);
    stop();
    // DEL

  }

  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void gober() {
    setVoltage(4);
  }

  public void stop() {
    setVoltage(0);
  }

  public void convoyer() {

    setVoltage(2);

  }

  public void convoyerLent() {

    setVoltage(1);
  }

}
