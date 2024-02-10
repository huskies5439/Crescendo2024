// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LancerAmpli extends SequentialCommandGroup {



  /** Creates a new LancerAmpli. */
  public LancerAmpli(Echelle echelle, Lanceur lanceur, Superstructure superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        echelle.setPIDCommand(0.2), 

        new SequentialCommandGroup(
          new WaitUntilCommand(echelle::atCible),

          new ParallelRaceGroup(
            lanceur.setPIDCommand(5),

            new DetecterNoteLancer(superstructure)
          )
        )
      ),
       Commands.runOnce(superstructure::setModeSpeaker)
       //echelle.setPIDCommand(0).until(echelle::isPositionDepart)


    );
  }
}
