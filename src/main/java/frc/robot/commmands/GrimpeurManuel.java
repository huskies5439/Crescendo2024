// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grimpeur;

public class GrimpeurManuel extends Command {
  Grimpeur grimpeur;
  DoubleSupplier voltage;

  /** Creates a new GrimpeurManuel. */
  public GrimpeurManuel(DoubleSupplier voltage, Grimpeur grimpeur) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grimpeur = grimpeur;
    this.voltage = voltage;
    addRequirements(grimpeur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (voltage.getAsDouble() >= 0.0) {
      if (!grimpeur.maxHauteur()) {
        grimpeur.setVoltage(voltage.getAsDouble());
      }
    } 
    else {
      if (!grimpeur.minHauteur()) {
        grimpeur.setVoltage(voltage.getAsDouble());
      }
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
