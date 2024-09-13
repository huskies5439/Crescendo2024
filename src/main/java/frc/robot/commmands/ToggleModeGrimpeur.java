// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;

public class ToggleModeGrimpeur extends Command {

  Superstructure superstructure;
  Mode modeActuel;

  public ToggleModeGrimpeur(Superstructure superstructure) {
    this.superstructure = superstructure;
    //IL NE FAUT PAS REQUIERT SUPERSTRUCTURE
  }

  @Override
  public void initialize() {
    modeActuel = superstructure.getMode();//On enregistre le mode actuel pour y retourner si on quitte le mode Grimpeur
    superstructure.setModeGrimpeur();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (modeActuel == Mode.AMPLI){
      superstructure.setModeAmpli();
    }
    else {
      superstructure.setModeSpeaker();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
