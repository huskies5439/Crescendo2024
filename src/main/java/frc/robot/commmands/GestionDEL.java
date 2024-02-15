// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;
import frc.robot.subsystems.Superstructure.PositionNote;

public class GestionDEL extends Command {

  private Superstructure superstructure;
  private int compteur;
  private boolean blink;

  public GestionDEL(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);

  }

  @Override
  public void initialize() { 
    compteur = 0;
    blink = false; 
  }

  @Override
  public void execute() {
   /* if (superstructure.getPositionNote() == PositionNote.AUCUNE) {//Pas de notes = fermée
      superstructure.closeDel();
    }

    else if (superstructure.getMode() == Mode.SPEAKER) {//Mode Speaker = Couleur constante
      superstructure.setCouleur(Color.kPurple);
    }

    else if (superstructure.getMode() == Mode.AMPLI) {//Mode Ampli = Blink

      compteur ++ ;

      if (compteur >= 100){//Période de blink à déterminer
        blink = !blink;
        compteur = 0;
      }

      if (blink) {
        superstructure.setCouleur(Color.kPurple);
      } 

      else {
        superstructure.closeDel();
      }

    } */
    superstructure.setCouleur(Color.kPaleTurquoise);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
