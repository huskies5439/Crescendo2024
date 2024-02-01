// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Grimpeur extends SubsystemBase {
  private TalonFX moteurGauche = new TalonFX(1);
  private TalonFX moteurDroit = new TalonFX(2);
  
  private double conversion = 1;
  private double maxPositionGrimpeur = 0.5; //mètres
  /** Creates a new Grimper. */
  public Grimpeur() {
    
    moteurGauche.setInverted(false);
    moteurGauche.setNeutralMode(NeutralModeValue.Brake);

    moteurDroit.setInverted(true);
    moteurDroit.setNeutralMode(NeutralModeValue.Brake);
    resetEncodeur();
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position Grimper Droite", getPositionDroit());
    SmartDashboard.putNumber("Position Grimper Gauche", getPositionGauche());

  }


  ////////CONSIGNES AUX MOTEURS
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
  


  /////////ENCODEURS
  public double getPositionDroit() {
    return moteurDroit.getPosition().getValueAsDouble() * conversion;
  }

  public double getPositionGauche(){
    return moteurGauche.getPosition().getValueAsDouble() * conversion;
  }

  public void resetEncodeur() {
    moteurGauche.setPosition(0);
    moteurDroit.setPosition(0);
  }



  //Proposition 1 pour la TEAM Grimpeur

  public boolean maxHauteurG(){
    return getPositionGauche() >= maxPositionGrimpeur;
  }

  public boolean minHauteurG(){
    return getPositionGauche() < 0;
  }

  public Command monterGauche(){
    return new StartEndCommand(() -> this.setVoltageGauche(3), this::stop, this).until(this::maxHauteurG);
  }

  public Command descendreGauche(){
    return new StartEndCommand(()->this.setVoltageGauche(-3), this::stop, this).until(this::minHauteurG);
  }


  //////puis faire l'équivalente pour la droite.....


}
