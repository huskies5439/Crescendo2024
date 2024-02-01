// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grimpeur;
import frc.robot.Constants;

public class Ajuster extends Command {

  private Grimpeur grimpeur;
  private int moteur;
  private double voltage;

  /** Creates a new Ajuster. */
  public Ajuster(Grimpeur grimpeur, int moteur) {
    this.voltage = Constants.ModuleConstants.grimpeurVoltage;
    this.grimpeur = grimpeur;
    this.moteur = moteur;
    addRequirements(grimpeur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (this.moteur == 1) {
      if (grimpeur.getPositionGauche() <= 0){
        grimpeur.setVoltageGauche(0);
      }else {
        grimpeur.setVoltageGauche(voltage);
      }

    }
    else if (this.moteur == 2) {
      if (grimpeur.getPositionDroit() <= 0){
        grimpeur.setVoltageDroit(0);
      }else {
        grimpeur.setVoltageDroit(voltage);
      }
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
    return false;
  }
}
