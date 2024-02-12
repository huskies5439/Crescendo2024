// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Echelle;



public class Homing extends SequentialCommandGroup {

  public Homing( Echelle echelle) {
   

    //Commande pour faire un Homing de l'échelle.
    addCommands( 
      //Avancer légèrement, puis on recule jusqu'à l'interrupteur
      Commands.run(()-> echelle.setVoltage(2), echelle).withTimeout(0.5), 
      Commands.run(()-> echelle.setVoltage(-1), echelle).until(echelle::isPositionDepart),
      Commands.runOnce(echelle::stop ),
      Commands.runOnce(echelle::resetEncodeur ),
       

       //On réavance, puis on recule à nouveau vers l'interrupteur pour valider. 
       Commands.run(()-> echelle.setVoltage(1), echelle). withTimeout(0.5), 
       Commands.run(()-> echelle.setVoltage(-1), echelle).until(echelle::isPositionDepart), 
       Commands.runOnce(echelle::stop ), 
       Commands.runOnce(echelle::resetEncodeur )
       
    );
 




    
    
    
    
    
    
    

  }
}
