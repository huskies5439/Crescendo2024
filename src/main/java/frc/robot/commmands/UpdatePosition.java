// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commmands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Limelight;

public class UpdatePosition extends Command {

  BasePilotable basePilotable;
  Limelight limelight;

  public UpdatePosition(BasePilotable basePilotable, Limelight limelight) {

    this.basePilotable = basePilotable;
    this.limelight = limelight;
    addRequirements(limelight);

  }

  @Override
  public void initialize() {
    limelight.setAlliance();
  }

  @Override
  public void execute() {//Critères à ajuster pour avoir une lecture fiable de AprilTags
    if(limelight.getTv() && limelight.getTa() > 0.12){
      basePilotable.addVisionMeasurement(limelight.getVisionPosition(), limelight.getTotalLatency() / 1000.0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
