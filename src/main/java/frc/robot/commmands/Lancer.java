// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lanceur;

public class Lancer extends Command {
  Lanceur lanceur;
  int vitesse;

  public Lancer(int vitesse,Lanceur lanceur) {
    this.lanceur = lanceur;
    this.vitesse = vitesse;
    addRequirements(lanceur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lanceur.setVitessePID(vitesse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
