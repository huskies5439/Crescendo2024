// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;



public class PreparerAmpli extends SequentialCommandGroup {


  public PreparerAmpli(Gobeur gobeur, Lanceur lanceur, Superstructure superstructure) {
    
    addCommands(       
      gobeur.convoyer() //Fait tourner lentement le gobeur et le lanceur pour transférer l'anneau dans le lanceur
      .alongWith(lanceur.setPIDCommand(8))//vitesse à déterminer
      .until(superstructure::isNoteDansLanceur), //Voir la discussion sur les lambdas dans WPILIB
      
       Commands.runOnce(superstructure::setModeAmpli)//Le robot est en mode ampli
      
      );
  }
}
