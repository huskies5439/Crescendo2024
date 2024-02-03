// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lanceur;

public class DetecterNoteLancer extends Command {
  Lanceur lanceur;

  boolean etatActuel;

  boolean etatPasse;
  int transitionEtat;

  /** Creates a new DetecterNoteLancer. */
  public DetecterNoteLancer(Lanceur lanceur) {
    this.lanceur = lanceur;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    etatActuel = false;

    etatPasse = false;

    transitionEtat = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    etatActuel = lanceur.notePresente();

    // true = on ne voit pas la note
    // false = on voit la note
    if (etatActuel != etatPasse) {

      if (!etatActuel) {

        transitionEtat++;
      }
    }

    etatPasse = etatActuel;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return transitionEtat >= 2;
  }

}
