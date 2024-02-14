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

  /** Creates a new ToggleModeGrimpeur. */
  public ToggleModeGrimpeur() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.modeActuel = superstructure.getMode();
    superstructure.setModeGrimpeur();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (this.modeActuel == Mode.AMPLI){
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
