// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.PositionNote;


public class PreparerAmpli extends SequentialCommandGroup {


  public PreparerAmpli( Echelle echelle, Gobeur gobeur, Lanceur lanceur, Superstructure superstructure) {
    
    addCommands( 
      
      new PIDechelle(0, echelle).until(echelle::isPositionDepart), // retracte l'échelle 
      
      Commands.startEnd(gobeur::convoyerLent, gobeur::stop, gobeur) //Fait tourner lentement le gobeur et le lanceur pour transférer l'anneau dans le lanceur
              .alongWith(Commands.startEnd(()->lanceur.setVoltage(2), lanceur:: stop,lanceur))//Transformer setVoltage en commande PID
              .until(()-> { return superstructure.getPositionNote() == PositionNote.LANCEUR; }) //Voir la discussion sur les lambdas dans WPILIB
      
      
  
      
      
      );
  }
}
