// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gobeur;

public class Gober extends Command {

  private Gobeur gobeur;

  
  public Gober(Gobeur gobeur) {
    
    this.gobeur = gobeur;
   
    addRequirements(gobeur);
  }

  @Override
  public void initialize() {  
  }

  @Override
  public void execute() { 
    if (!gobeur.notePresente() /* && !lanceur.notePresente()*/) { //Il n'y a pas de notes dans le gobeur. Ni le lanceur. Valider la syntaxe pour le lanceur
      gobeur.gober();
    } else {
      gobeur.stop();
    }
    // gobeur.gober();
  }

  @Override
  public void end(boolean interrupted) {
    gobeur.stop();

  }

  @Override
  public boolean isFinished() {
    return gobeur.notePresente();
  }
}
