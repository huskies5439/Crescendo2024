// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;


public class LancerSpeaker extends SequentialCommandGroup {

  public LancerSpeaker(Gobeur gobeur, Lanceur lanceur, Superstructure superstructure) {

    addCommands(
        new ParallelRaceGroup(

                lanceur.setPIDCommand(40), // valeur vcible a vérifier

                Commands.waitUntil(lanceur::atCible)//Quand le lanceur a atteint sa cible, on envoit la note dans le lanceur
                        .andThen(gobeur.convoyer())

                // new DetecterNoteLancer(superstructure)//Met fin à la commande
                )
              );
  }
}
