// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrimpeurV2 extends SubsystemBase {
  private TalonFX moteur;

  private double conversion = 1; //à déterminer avec la team conception
  private double maxPositionGrimpeur = 0.5; //valeur à déterminer, en mètres
  private double voltage = 3; //valeur à déterminer

  String dashName;
  
  public GrimpeurV2(int CANID, boolean inverted, String dashName) {
    moteur = new TalonFX(CANID);

    moteur.setInverted(inverted);
    moteur.setNeutralMode(NeutralModeValue.Brake);

    resetEncodeur();
    this.dashName = dashName;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Grimpeur "+dashName+" Position", getPosition());
  }

  public void setVoltage(boolean monter){
    if (!monter){
      voltage = - voltage;
    }

    moteur.set(voltage);
  }

  public void stop() {
    moteur.set(0);
  }


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

  public Command monter(){
    return this.startEnd(()-> this.setVoltage(true), this::stop).until(this::maxHauteur);//requiert this
  }

  public Command descendre(){
    return  this.startEnd(()-> this.setVoltage(false), this::stop).until(this::minHauteur);
  }

}
