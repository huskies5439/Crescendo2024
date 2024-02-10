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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LancerSpeaker extends SequentialCommandGroup {
  /** Creates a new LancerSpeacker. */
  public LancerSpeaker(Echelle echelle, Gobeur gobeur, Lanceur lanceur, Superstructure superstructure) {

    addCommands(

        //echelle.setPIDCommand(0).until(echelle::isPositionDepart), // retracte l'échelle

        new ParallelRaceGroup(
                lanceur.setPIDCommand(20), // valeur vcible a vérifier

                Commands.waitUntil(lanceur::atCible)
                        .andThen(gobeur.convoyer()),

                new DetecterNoteLancer(superstructure)
                )
              );
  }
}
