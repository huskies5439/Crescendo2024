// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Echelle;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Homing extends SequentialCommandGroup {


  /** Creates a new Homing. */
  public Homing( Echelle echelle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      (Commands.run(()-> echelle.setVoltage(2), echelle). withTimeout(0.5)), 
      (Commands.run(()-> echelle.setVoltage(-1), echelle).until(echelle::isPositionDepart)),
      (Commands.runOnce(echelle::stop )),
      (Commands.runOnce(echelle::resetEncodeur )),
       

       //On réavance, puis on recule à nouveau vers l'interrupteur pour valider. 

       (Commands.run(()-> echelle.setVoltage(1), echelle). withTimeout(0.5)), 
       (Commands.run(()-> echelle.setVoltage(-1), echelle).until(echelle::isPositionDepart)), 
       (Commands.runOnce(echelle::stop )), 
       (Commands.runOnce(echelle::resetEncodeur ))
       
    );
 




    
    
    
    
    
    
    

  }
}
