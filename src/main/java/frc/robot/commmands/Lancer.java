// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lanceur;

public class Lancer extends Command {
  Lanceur lanceur;
  int vitesse;

  //Il faudra éventuellement ajouter une logique pour la fin de la commande.
  public Lancer(int vitesse,Lanceur lanceur) {
    this.lanceur = lanceur;
    this.vitesse = vitesse;
    addRequirements(lanceur);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    lanceur.setVitessePID(vitesse);
  }

  @Override
  public void end(boolean interrupted) {
    lanceur.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
