// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Lanceur extends SubsystemBase {

  private final TalonFX moteurG = new TalonFX(1);
  private final TalonFX moteurD = new TalonFX(2);


  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0); // Trouver les valeurs du
                                                                                       // feedforward et du pid
  private PIDController pidG = new PIDController(0, 0, 0);
  private PIDController pidD = new PIDController(0, 0, 0);

  private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
  private GenericEntry valeurLanceurCible = calibration.add("valeur lanceur cible", 0).getEntry();
  

  /** Creates a new Lanceur. */
  public Lanceur() {

    moteurG.setInverted(true);
    moteurD.setInverted(false);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("VitesseG", getVitesseG());
    SmartDashboard.putNumber("VitesseD", getVitesseD());

  }

  public void setVoltage(double volts) {
    moteurG.setVoltage(volts);
    moteurD.setVoltage(volts);

  }

  public void stop() {
    setVoltage(0);

  }

  public double getPositionG() {
    return moteurG.getPosition().getValueAsDouble();// en rotation
  }

  public double getVitesseG() {
    return moteurG.getVelocity().getValueAsDouble();// en rotation par seconde
  }

  public void setVitessePIDGauche(double vcible) {
    moteurG.setVoltage(pidG.calculate(getVitesseG(), vcible) + feedforward.calculate(vcible));
  }

  public double getPositionD() {
    return moteurD.getPosition().getValueAsDouble();
  }

  public double getVitesseD() {
    return moteurD.getVelocity().getValueAsDouble();

  }

  public void setVitessePIDDroite(double vcible) {
    moteurD.setVoltage(pidD.calculate(getVitesseD(), vcible) + feedforward.calculate(vcible));
  }

  public void setVitessePID(double vcible) {
    setVitessePIDDroite(vcible);
    setVitessePIDGauche(vcible);

  }

  public double getValeurShuffleboard() {
    return valeurLanceurCible.getDouble(0);
  }

  public void setVoltageShuffleboard() {
    setVoltage(getValeurShuffleboard());
  }

  public void setPIDShuffleboard() {
    setVitessePID(getValeurShuffleboard());
  }
public void setPID(double vrotation){
    setVitessePIDDroite(vrotation);
    setVitessePIDGauche(vrotation);
}


}
