// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ChiffreMagique;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;

public class LancerSpeaker extends ParallelCommandGroup {

  //Commande qui ne se termine pas
  //Utiliser un decorateur timeout en auto
  public LancerSpeaker(Gobeur gobeur, Lanceur lanceur) {

    addCommands(

        lanceur.setPIDCommand(ChiffreMagique.vitesseLancerSpeaker),

        Commands.waitUntil(lanceur::atCible)// Quand le lanceur a atteint sa cible, on envoit la note dans le lanceur
            .andThen(gobeur.convoyerCommand())
    );
  }
}
