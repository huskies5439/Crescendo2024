// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {

  private final TalonFX moteurG = new TalonFX(1);
  private final TalonFX moteurD = new TalonFX(2);

  private final SimpleMotorFeedforward feedforwardG = new SimpleMotorFeedforward(0,0); // Trouver les valeurs du feedforward et du pid
  private PIDController pidG = new PIDController(0, 0, 0);
  
private final SimpleMotorFeedforward feedforwardD = new SimpleMotorFeedforward(0,0); // Trouver les valeurs du feedforward et du pid
  private PIDController pidD = new PIDController(0, 0, 0);
  

  /** Creates a new Lanceur. */
  public Lanceur() {
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVitesseSimple(double volts){
    moteurG.setVoltage(volts);
    moteurD.setVoltage(volts);

  }

  public double getVitesseG(){
    return moteurG.getVelocity().getValue();

  }

  public void setVitessePIDG(double vcible) {
    moteurG.setVoltage(pidG.calculate(getVitesseG(),vcible)+feedforwardG.calculate(vcible));
  }


  public double getVitesseD(){
    return moteurD.getVelocity().getValue();

  }

  public void setVitessePIDD(double vcible) {
    moteurD.setVoltage(pidD.calculate(getVitesseD(),vcible)+feedforwardD.calculate(vcible));
  }
}
