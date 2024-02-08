// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.PositionNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreparerAmpli extends SequentialCommandGroup {
  /** Creates a new PreparerAmpli. */
  public PreparerAmpli( Echelle echelle, Gobeur gobeur, Lanceur lanceur, Superstructure superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()); 
    addCommands( 
      
      new PIDechelle(0, echelle).until(echelle::isPositionDepart), // retracte l'échelle 
      
      Commands.startEnd(gobeur::convoyerLent, gobeur::stop, gobeur)
            .alongWith(Commands.startEnd(()->lanceur.setVoltage(2), lanceur:: stop,lanceur))
            .until(()-> superstructure.ouEstLaNote(PositionNote.LANCEUR))
      
      
      
      
      
      );
  }
}
