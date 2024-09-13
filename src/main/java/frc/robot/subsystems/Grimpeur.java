// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Grimpeur extends SubsystemBase {
  
  private TalonFX moteur;

/* Pignon 14 dents sur le falcon fait tourner gear 40 dents. La gear 40 dents est solidaire d'une
    gear 14 dents (même vitesse). La gear 14 dents fait tourner une gear 80 dents. La gear 80 dents fait tourner 
    une poulie de 0.75 pouces auquelle s'enroule d'une corde */
  private double conversion  = (14.0/40)*(14.0/80)*Units.inchesToMeters(0.75)*Math.PI;


  private double maxPositionGrimpeur = 0.4;

  private double voltageRapide = 3;
  private double voltageLent = 0.25;

  String dashName;
  
  public Grimpeur(int CANID, boolean inverted, String dashName) {
    moteur = new TalonFX(CANID);

    moteur.setInverted(inverted);
    moteur.setNeutralMode(NeutralModeValue.Brake);

    resetEncodeur();

    this.dashName = dashName;
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("Grimpeur "+dashName+" Position", getPosition());
  }

  //Consignes au moteur
  public void setVoltage(double voltage){
    moteur.set(voltage);
  }

  public void stop() {
    setVoltage(0);
  }


  //Encodeur
  public double getPosition(){
    return moteur.getPosition().getValueAsDouble()*conversion;
  }

  public void resetEncodeur(){
    moteur.setPosition(0);
  }

  public  boolean maxHauteur(){
    return getPosition()>= maxPositionGrimpeur;
  }

  public boolean minHauteur(){
    return getPosition() <=0;
  }


  //Commandes pour monter et descendre
  //Parfois il faut aller rapidement, parfois rapidement.
  public Command monterCommandSansLimite(boolean rapide){
    return this.startEnd(()-> this.setVoltage(rapide ? voltageRapide : voltageLent), this::stop);
  }

  public Command descendreCommandSansLimite(boolean rapide){
    return this.startEnd(()-> this.setVoltage(rapide ? -voltageRapide : -voltageLent), this::stop);
  }


  //Commandes pour monter et descendre en s'arrêtant automatiquement aux limites du grimpeur
  public Command monterCommand(boolean rapide){
    return monterCommandSansLimite(rapide).until(this::maxHauteur);
  }

  public Command descendreCommand(boolean rapide){
    return descendreCommandSansLimite(rapide).until(this::minHauteur);
  }
  

}
