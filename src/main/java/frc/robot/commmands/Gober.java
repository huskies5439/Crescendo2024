// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gobeur;

public class Gober extends Command {

  private Gobeur gobeur;
  /** Creates a new Gober. */
  public Gober(Gobeur gobeur) {
    
    this.gobeur = gobeur;
   
    addRequirements(gobeur);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (gobeur.notePresente()) { 
      gobeur.gober();
    } else {
      gobeur.stop();
    }
    // gobeur.gober();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gobeur.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
