// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grimpeur extends SubsystemBase {
  private TalonFX moteur = new TalonFX(80);
  private TalonFX moteurFollower = new TalonFX(81);
  
  /** Creates a new Grimper. */
  public Grimpeur() {
    moteurFollower.setControl(new Follower(80, false));
    moteur.setInverted(false);
    moteur.setNeutralMode(NeutralModeValue.Brake);
    // moteur.configselectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
  
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
