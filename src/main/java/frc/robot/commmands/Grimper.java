// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grimpeur;
import frc.robot.Constants;

public class Grimper extends Command {
  /** Creates a new Grimper. */

  private Grimpeur grimpeur;
  private double voltage; 
  private double maxHauteur; 


  public Grimper(Grimpeur grimpeur) {
      
    this.grimpeur = grimpeur;
    this.voltage = Constants.grimpeurVoltage;
    this.maxHauteur = 10.0;
    addRequirements(grimpeur); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (grimpeur.getPositionGauche() >= this.maxHauteur){
      grimpeur.setVoltageGauche(0);
    }else {
      grimpeur.setVoltageGauche(voltage);
    }


  
    if (grimpeur.getPositionDroit() >= this.maxHauteur){
      grimpeur.setVoltageDroit(0);
    }else {
      grimpeur.setVoltageDroit(voltage);
    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    grimpeur.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
     if (grimpeur.getPositionDroit() >= this.maxHauteur && grimpeur.getPositionGauche() >= this.maxHauteur ){return true;}
    return false;
  }
}
