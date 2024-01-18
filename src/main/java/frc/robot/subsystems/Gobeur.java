// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gobeur extends SubsystemBase { 

  private final TalonFX moteur = new TalonFX(5);
  private final DigitalInput infrarouge = new DigitalInput(0);
  private AddressableLED del = new AddressableLED(0);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(8); // LE nombre de sections de DEL ici 3
                                                                        // DEL/Section
  /** Creates a new Gobeur. */
  public Gobeur() {
    moteur.setInverted(false);
    stop();
    // DEL
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
    setCouleur(Color.kOrange);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("infrarouge", getInfrarouge());
  }
  public void gober(){
    moteur.setVoltage(4);
  } 
  public void stop(){
    moteur.setVoltage(0);
  }
   
  public boolean getInfrarouge(){
    return infrarouge.get();
  }

  public void setCouleur(int rouge, int vert, int bleu) {
    for (var i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setRGB(i, rouge, bleu, vert);
    }
    del.setData(delBuffer);

  }

  public void setCouleur(Color color) {
    for (int i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setLED(i, color);
    }
    del.setData(delBuffer);

  }
}
