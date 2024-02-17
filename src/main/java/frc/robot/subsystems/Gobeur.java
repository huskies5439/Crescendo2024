// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gobeur extends SubsystemBase {

  private final TalonFX moteur = new TalonFX(6);

  public Gobeur() {
    moteur.setInverted(false);
    stop();
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


  //Commande pour convoyer l'anneau vers le lanceur.
  public Command convoyerCommand() {
    return this.startEnd(() -> this.setVoltage(3), this::stop);
  }

}
