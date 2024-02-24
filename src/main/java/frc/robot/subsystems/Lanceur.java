// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Lanceur extends SubsystemBase {

  private final TalonFX moteurG = new TalonFX(4);
  private final TalonFX moteurD = new TalonFX(5);


  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.124, 0.0); //Voir le SysID Crescendo
                                                                                      
  private PIDController pidG = new PIDController(0.1, 0, 0);
  private PIDController pidD = new PIDController(0.1, 0, 0);
  

  private final Debouncer debouncerLanceur = new Debouncer(0.1,DebounceType.kRising);


  public Lanceur() {

    moteurG.setInverted(true);
    moteurD.setInverted(false);

    moteurG.setNeutralMode(NeutralModeValue.Coast);
    moteurD.setNeutralMode(NeutralModeValue.Coast);
    
    pidG.setTolerance(3.0);//Fonctionne à 3, on test à 2
    pidD.setTolerance(3.0);

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
    pidG.setSetpoint(0);//Pour que la condition atCible arrête d'être vraie à la fin de la commande
    pidD.setSetpoint(0);
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
  public void setPID(double vcible) {
    setVitessePIDDroite(vcible);
    setVitessePIDGauche(vcible);

  }


  public Command setPIDCommand(double vcible){
    return this.runEnd(()-> this.setPID(vcible), this::stop); //RunEnd car le PID doit être dans le EXECUTE de la commande
       
  }

  public Command setPIDCommandSansFin(double vcible){
    return this.run(()-> this.setPID(vcible)); //RunEnd car le PID doit être dans le EXECUTE de la commande
       
  }


  public boolean atCible() {
    return debouncerLanceur.calculate(pidG.atSetpoint() && pidD.atSetpoint());

  }

}
