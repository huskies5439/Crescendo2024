// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Lanceur extends SubsystemBase {

  private final TalonFX moteurG = new TalonFX(4);
  private final TalonFX moteurD = new TalonFX(5);


  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.124, 0.0); // valeur du moteur gauche
                                                                                      
  private PIDController pidG = new PIDController(0.1, 0, 0);//a calibrer. Kp = 1 c'est trop
  private PIDController pidD = new PIDController(0.1, 0, 0);

  private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
  private GenericEntry valeurLanceurCible = calibration.add("valeur lanceur cible", 0).getEntry();
  

  /** Creates a new Lanceur. */
  public Lanceur() {

    moteurG.setInverted(true);
    moteurD.setInverted(false);

    moteurG.setNeutralMode(NeutralModeValue.Coast);
    moteurD.setNeutralMode(NeutralModeValue.Coast);
    
    pidG.setTolerance(3);//valeur a determiner en RPS
    pidD.setTolerance(3);//valeur a determiner en RPS

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("VitesseG", getVitesseG());
    SmartDashboard.putNumber("VitesseD", getVitesseD());
    SmartDashboard.putBoolean("Lanceur At Cible", atCible());

  }


  //Lancer simple
  public void setVoltage(double volts) {
    moteurG.setVoltage(volts);
    moteurD.setVoltage(volts);

  }

  public void stop() {
    setVoltage(0);
  }

  public Command commandeVoltageSimple(double voltage){
    return this.startEnd(()->this.setVoltage(voltage), this::stop);
  }

  //Shuffleboard
  public double getValeurShuffleboard() {
    return valeurLanceurCible.getDouble(0);
  }

  public void setVoltageShuffleboard() {
    setVoltage(getValeurShuffleboard());
  }

  //Encodeur et PID/FF Gauche
  public double getPositionG() {
    return moteurG.getPosition().getValueAsDouble();// en rotation
  }

  public double getVitesseG() {
    return moteurG.getVelocity().getValueAsDouble();// en rotation par seconde
  }

  public void setVitessePIDGauche(double vcible) {
    moteurG.setVoltage(pidG.calculate(getVitesseG(), vcible) + feedforward.calculate(vcible));
  }


  //Encodeur et PID/FF DROIT
  public double getPositionD() {
    return moteurD.getPosition().getValueAsDouble();
  }

  public double getVitesseD() {
    return moteurD.getVelocity().getValueAsDouble();

  }

  public void setVitessePIDDroite(double vcible) {
    moteurD.setVoltage(pidD.calculate(getVitesseD(), vcible) + feedforward.calculate(vcible));
  }

  //PID
  public void setVitessePID(double vcible) {
    setVitessePIDDroite(vcible);
    setVitessePIDGauche(vcible);

  }


  public Command setPIDCommand(double vcible){//Utiliser raceWith(DetecterLanceurNote) pour donner une condition de fin à cette commande
    return this.runEnd(()-> this.setVitessePID(vcible), this::stop); //RunEnd car le PID doit être dans le EXECUTE de la commande
       
  }

  public boolean atCible() {
    return pidG.atSetpoint() 
    && pidD.atSetpoint();

  }

}
