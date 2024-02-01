// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Grimpeur extends SubsystemBase {
  private TalonFX moteurGauche = new TalonFX(1);
  private TalonFX moteurDroit = new TalonFX(2);
  
  private double conversion = 1;
  /** Creates a new Grimper. */
  public Grimpeur() {
    moteurDroit.setControl(new Follower(80, false));
    moteurGauche.setInverted(false);
    moteurGauche.setNeutralMode(NeutralModeValue.Brake);
    resetEncodeur();
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position Grimper Droite", getPositionDroit());
    SmartDashboard.putNumber("Position Grimper Gauche", getPositionGauche());
  }

  public Double getPositionDroit() {
    return moteurDroit.getPosition().getValue() * conversion;
  }

  public Double getPositionGauche(){
    return moteurGauche.getPosition().getValue() * conversion;
  }



  public void resetEncodeur() {
    moteurGauche.setPosition(0);
  }

  public void setVoltageGauche(double volt) {
    moteurGauche.set(volt);
  }

  public void setVoltageDroit(double volt) {
    moteurDroit.set(volt);
  }

  public void stop() {
    setVoltageDroit(0);
    setVoltageGauche(0);

  }
}
