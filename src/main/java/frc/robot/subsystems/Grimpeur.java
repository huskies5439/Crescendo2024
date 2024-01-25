// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grimpeur extends SubsystemBase {
  private TalonFX moteur = new TalonFX(80);
  private TalonFX moteurFollower = new TalonFX(81);
  
  /** Creates a new Grimper. */
  public Grimpeur() {
    moteurFollower.setControl(new Follower(80, false));
    moteur.setInverted(false);
    moteur.setNeutralMode(NeutralModeValue.Brake);
    resetEncodeur();
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position Grimper",getPosition());
  }

  public double getPosition() {
    return moteur.getPosition().getValueAsDouble();
  }

  public void resetEncodeur() {
    moteur.setPosition(0);
  }

  public void setVoltage(double volt) {
    moteur.set(volt);
  }

  public void stop() {
    setVoltage(0);
  }
}
