// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.PositionNote;

public class Gober extends Command {

  private Gobeur gobeur;
  private Superstructure superstructure;

  public Gober(Gobeur gobeur, Superstructure superstructure) {

    this.gobeur = gobeur;
    this.superstructure = superstructure;
    addRequirements(gobeur);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (superstructure.getPositionNote() == PositionNote.AUCUNE) { // Il n'y a pas de notes dans le gobeur. Ni le lanceur.
      gobeur.setVoltage(3.5);
    } else {
      gobeur.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    gobeur.stop();

  }

  @Override
  public boolean isFinished() {
    return superstructure.getPositionNote() == PositionNote.GOBEUR;
  }
}
