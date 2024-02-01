// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Echelle;

public class PIDechelle extends Command {
  private Echelle echelle;
  private double cible;
  private ProfiledPIDController pid;
  private double voltage = 0;

  public PIDechelle(double cible, Echelle echelle) {
    this.echelle = echelle;
    this.cible = cible;
    addRequirements(echelle);
    pid = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(0.25, 0.5));
    pid.setTolerance(0.01);
  }

  @Override
  public void initialize() {
   cible = MathUtil.clamp(cible,0, 0.25);

   pid.setGoal(cible);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltage = pid.calculate(echelle.getPosition());
    echelle.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    echelle.stop();//par sécurité
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
