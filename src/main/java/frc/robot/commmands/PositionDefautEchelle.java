// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;

public class PositionDefautEchelle extends Command {

  Echelle echelle;
  Superstructure superstructure;

  public PositionDefautEchelle(Echelle echelle, Superstructure superstructure) {
    this.echelle = echelle;
    this.superstructure = superstructure;
    addRequirements(echelle);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    if (superstructure.getMode() == Mode.GRIMPEUR){
      echelle.setPositionPID(0.28);
      SmartDashboard.putString("etat echelle","grimpeur");
    }
    else
    {
      echelle.setPositionPID(0.0); 
       SmartDashboard.putString("etat echelle","rentre");
    }
  }


  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
