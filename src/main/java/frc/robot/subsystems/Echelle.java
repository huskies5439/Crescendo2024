// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Echelle extends SubsystemBase {
  private TalonFX moteur = new TalonFX(3);
  private DigitalInput limitSwitch = new DigitalInput(4);
  private double conversionEncodeur;
  /** Creates a new Echelle. */
  public Echelle() {

    moteur.setInverted(false);
    moteur.setNeutralMode(NeutralModeValue.Brake);
    resetEncodeur();
    
    /* Pignon 14 dents sur le falcon fait tourner gear 40 dents. La gear 40 dents est solidaire d'une
    gear 14 dents (même vitesse). La gear 14 dents fait tourner une gear 60 dents. La gear 60 dents est solidaire d'une roue dentée
    de 16 dents qui fait tourner la chaine 25. Chaque maille de la chaine fait 0.25 pouces*/
    conversionEncodeur = (14.0/40)*(14.0/60)*(16.0)*Units.inchesToMeters(0.25);


  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("Position Échelle", getPosition());
  SmartDashboard.putBoolean("Échelle à l'ampli", positionAmpli());
    SmartDashboard.putBoolean("Échelle position de départ", positionDepart());

    if (positionDepart()) {
      moteur.setPosition(0);
    }
  }
  public void resetEncodeur() {
    moteur.setPosition(0);   
  }

  public boolean positionDepart() {
    return limitSwitch.get();
  }

  public double getPosition() {
   return moteur.getPosition().getValueAsDouble()*conversionEncodeur;
  }

  public void setVoltage(double volt) {
    setVoltage(volt);
  }

  public double getVitesse() {
   return moteur.getVelocity().getValueAsDouble()*conversionEncodeur;
  }

  public void stop() {
    setVoltage(0);
  }

  public boolean positionAmpli() {
    return getPosition()>0.2; //valeur à déterminer
  }
}

