// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commmands;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LancerGenerique extends ParallelRaceGroup {

  public LancerGenerique(int vitesse, Lanceur lanceur,Superstructure  superstructure) {
    
   
    addCommands(
          Commands.runEnd(()->lanceur.setPID(vitesse), lanceur::stop, lanceur, lanceur),
          new DetecterNoteLancer(superstructure)
         );
  }
}
