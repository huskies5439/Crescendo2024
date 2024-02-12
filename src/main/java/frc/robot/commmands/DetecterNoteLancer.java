// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class DetecterNoteLancer extends Command {
  Superstructure superstructure; 
  boolean etatActuel;

  boolean etatPasse;
  int transitionEtat;

  public DetecterNoteLancer(Superstructure superstructure) {
    this.superstructure = superstructure;

  }

  @Override
  public void initialize() {
    etatActuel = false;

    etatPasse = false;

    transitionEtat = 0;
  }

  @Override
  public void execute() {
    etatActuel = superstructure.isNoteDansLanceur();

    // true = on ne voit pas la note
    // false = on voit la note
    if (etatActuel != etatPasse) {

      if (!etatActuel) {

        transitionEtat++;
      }
    }

    etatPasse = etatActuel;
    SmartDashboard.putNumber("Détetecter Note, monte de 2 par Note", transitionEtat);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return transitionEtat >= 2;
  }

}
