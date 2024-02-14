// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Echelle extends SubsystemBase {

  private TalonFX moteur = new TalonFX(3);

  private DigitalInput limitSwitch = new DigitalInput(4);

  private ProfiledPIDController pid = new ProfiledPIDController(200, 0, 0,  //Valeur faible pour premiers essaies
        new TrapezoidProfile.Constraints(0.5, 1.0)); //Valeur Charged Up : Kp : 0.56, Vmax 0.25, Amax 0.5


  private double conversionEncodeur;

  private double maxEchelle = 0.28; 


  public Echelle() {

    moteur.setInverted(false);
    moteur.setNeutralMode(NeutralModeValue.Brake);

    resetEncodeur();

    pid.setTolerance(0.01);


    /* Pignon 14 dents sur le falcon fait tourner gear 40 dents. La gear 40 dents est solidaire d'une
    gear 14 dents (même vitesse). La gear 14 dents fait tourner une gear 60 dents. La gear 60 dents est solidaire d'une roue dentée
    de 16 dents qui fait tourner la chaine 25. Chaque maille de la chaine fait 0.25 pouces*/
    conversionEncodeur = (14.0/40)*(14.0/60)*(16.0)*Units.inchesToMeters(0.25);


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position Échelle", getPosition());
    SmartDashboard.putBoolean("Échelle à la cible", atCible());
    SmartDashboard.putBoolean("Échelle position de départ", isPositionDepart());


    //Reset encodeur quand on redescend on active la limit switch
    if (isPositionDepart()) {
      resetEncodeur();
    }

  }

  //Envoyer consigne au moteur
  public void setVoltage(double volt) {
    moteur.setVoltage(volt);
  }

  public void stop() {
    setVoltage(0);
  }


  //Encodeurs
  public double getPosition() {
   return moteur.getPosition().getValueAsDouble()*conversionEncodeur;
  }

  public double getVitesse() {
   return moteur.getVelocity().getValueAsDouble()*conversionEncodeur;
  }

  public void resetEncodeur() {
    moteur.setPosition(0);   
  }


  //Boolean donne la valeur de la limit Switch
  public boolean isPositionDepart() {
    return !limitSwitch.get(); //devrait être positif quand la switch est cliquée
  }


  // PID
  public void setPositionPID(double cible){
    
    cible = MathUtil.clamp(cible,0, maxEchelle);
    if (cible == 0 && getPosition() < 0.01){
      if (isPositionDepart()){
        stop();
      }
      else{
        setVoltage(-1.0);
      }
    }
    else{
      double voltagePID = pid.calculate(getPosition(), cible);
      setVoltage(voltagePID);
    }
  }

  public boolean atCible(){
    return pid.atGoal();//Tiens en compte la tolérance
  }

  //Commande PID de l'échelle pouvant être appelée directement
  public Command setPIDCommand(double cible){
    return this.runEnd(()-> this.setPositionPID(cible), this::stop);

  }

 
}

