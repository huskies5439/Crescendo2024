// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gobeur extends SubsystemBase { 

  private final TalonFX moteur = new TalonFX(5);
  
  /** Creates a new Gobeur. */
  public Gobeur() {
    moteur.setInverted(false);
    stop();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void gober(){
    moteur.setVoltage(3);

  } 
  public void stop(){
    moteur.setVoltage(0);

  }

}
