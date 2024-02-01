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


  //Cette commande ne se termine jamais par elle-même. À voir si cela va éventuellement causer des problèmes
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

  @Override
  public void execute() {
    voltage = pid.calculate(echelle.getPosition());
    echelle.setVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    echelle.stop();//par sécurité
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
