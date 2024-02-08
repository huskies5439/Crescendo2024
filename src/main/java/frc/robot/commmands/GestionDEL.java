// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;
import frc.robot.subsystems.Superstructure.PositionNote;

public class GestionDEL extends Command {
  /** Creates a new GestionDEL. */
  private Superstructure superstructure;
  private int compteur;

  public GestionDEL(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.superstructure = superstructure;
    addRequirements(superstructure);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    compteur = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    compteur ++ ;

    if (superstructure.getPositionNote() == PositionNote.AUCUNE) {
      superstructure.closeDel();
    }

    else if (superstructure.getMode() == Mode.AMPLI) {

      if (compteur <= 50) {
        superstructure.closeDel();
        
      } else if (compteur >= 51) {
        superstructure.setCouleur(Color.kPurple);
      }

      if (compteur == 100) {
        compteur = 0;
      }

    } else if (superstructure.getMode() == Mode.SPEAKER) {
      superstructure.setCouleur(Color.kPurple);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
