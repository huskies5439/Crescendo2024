// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;


public class LancerAmpli extends ParallelCommandGroup {

  public LancerAmpli(Echelle echelle, Lanceur lanceur, Gobeur gobeur) {

  //Commande qui ne se termine pas
  //Utiliser un decorateur timeout en auto
    addCommands(

        //Sortir l'échelle
        echelle.setPIDCommand(0.2), 
        //Faire tourner le convoyeur pour décoincer la note
        gobeur.convoyerCommand(),

        //Attendre que l'échelle soit sortie, puis on lance
        new SequentialCommandGroup(
          new WaitCommand(0.4),//Ajuster si l'échelle n'est pas assez déployée 
          lanceur.setPIDCommand(8)    
        )
    );
  }
}
