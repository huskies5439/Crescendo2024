// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {

  private final CANSparkMax moteurG = new CANSparkMax(10, MotorType.kBrushless);  
  private final CANSparkMax moteurD = new CANSparkMax(9, MotorType.kBrushless);

  private final double conversionEncodeur = 1;//à déterminer
 

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0); // Trouver les valeurs du feedforward et du pid
  private PIDController pidG = new PIDController(0, 0, 0);
  private PIDController pidD = new PIDController(0, 0, 0);


  private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
  private GenericEntry valeurLanceurCible = calibration.add("valeur lanceur cible",0).getEntry();
  


  /** Creates a new Lanceur. */
  public Lanceur() {
    moteurG.setInverted(true);
    moteurD.setInverted(false);

    moteurG.getEncoder().setPositionConversionFactor(conversionEncodeur);
    moteurG.getEncoder().setVelocityConversionFactor(conversionEncodeur/60); //pour obtenir des rotation par seconde


    moteurD.getEncoder().setPositionConversionFactor(conversionEncodeur);
    moteurD.getEncoder().setVelocityConversionFactor(conversionEncodeur/60); //pour obtenir des rotation par seconde

  

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("VitesseG", getVitesseG());
    SmartDashboard.putNumber("VitesseD", getVitesseD());    

  }

  public void setVoltage(double volts){
    moteurG.setVoltage(volts);
    moteurD.setVoltage(volts);


  }

  public void stop(){
    setVoltage(0);

  }

  public double getPositionG(){
    return moteurG.getEncoder().getPosition();//en rotation
  }

  public double getVitesseG(){
    return moteurG.getEncoder().getVelocity();//en rotation par seconde
  }

  public void setVitessePIDGauche(double vcible) {
    moteurG.setVoltage(pidG.calculate(getVitesseG(),vcible)+feedforward.calculate(vcible));
  }

  public double getPositionD(){
    return moteurD.getEncoder().getPosition();
  }

  public double getVitesseD(){
    return moteurD.getEncoder().getVelocity();

  }

  public void setVitessePIDDroite(double vcible) {
    moteurD.setVoltage(pidD.calculate(getVitesseD(),vcible)+feedforward.calculate(vcible));
  }

  public void setVitessePID(double vcible){
      setVitessePIDDroite(vcible);
      setVitessePIDGauche(vcible);

  }

  public double getValeurShuffleboard() {
    return valeurLanceurCible.getDouble(0);
}

public void setVoltageShuffleboard(){
  setVoltage(getValeurShuffleboard());
}

public void setPIDShuffleboard(){
  setVitessePID(getValeurShuffleboard());
}




}
