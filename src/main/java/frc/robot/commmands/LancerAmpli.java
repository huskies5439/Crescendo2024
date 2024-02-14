// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;


public class LancerAmpli extends SequentialCommandGroup {

  public LancerAmpli(Echelle echelle, Lanceur lanceur, Superstructure superstructure, Gobeur gobeur) {
   

    addCommands(

      //Sortir l'échelle et lancer lentement
      new ParallelRaceGroup(
        echelle.setPIDCommand(0.2), //hauteur à valider
        
        gobeur.convoyer(),

        //Attendre que l'échelle soit sortie, puis on lance
        new SequentialCommandGroup(
          new WaitUntilCommand(echelle::atCible),

          //On lance jusqu'à ce que la note sorte du lanceur
          new ParallelRaceGroup(
            lanceur.setPIDCommand(20),//Valeur à déterminer
            new DetecterNoteLancer(superstructure)//Termine le Race, qui termine le Sequential, qui termine le Race
          )
        )
      ),

       Commands.runOnce(superstructure::setModeSpeaker)
    );
  }
}
