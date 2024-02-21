// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  int rainbowFirstPixelHue = 0;

  public enum Mode {// Trois etats pour savoir comment est actuellement géré la note dans le robot
    SPEAKER,
    AMPLI,
    GRIMPEUR
  }

  public enum PositionNote {// Trois positions de note possibles
    AUCUNE,
    GOBEUR,
    LANCEUR
  }

  private Mode mode = Mode.SPEAKER;

  private PositionNote positionNote = PositionNote.AUCUNE;

  private final DigitalInput capteurGobeur = new DigitalInput(0); // Émetteur branché sur PWM 0
  private final DigitalInput capteurLanceur = new DigitalInput(2); // Émetteur branché sur PWM 1

  private AddressableLED del = new AddressableLED(9); // Port PWM
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(23);

  /** Creates a new Superstructure. */
  public Superstructure() {

    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
    setModeSpeaker();
  }

  @Override
  public void periodic() {

    switch (positionNote) {

      case AUCUNE:// Si aucune note, on valide si un des capteurs detecte quelque chose pout
                  // indiquer que la note est présente
        if (isNoteDansGobeur()) {
          positionNote = PositionNote.GOBEUR;
        } else if (isNoteDansLanceur()) {
          positionNote = PositionNote.LANCEUR;
        }
        break;

      case GOBEUR:

        if (isNoteDansLanceur()) {// On valide ou est rendu la note
          positionNote = PositionNote.LANCEUR;
        } else if (!isNoteDansGobeur()) { // Si gobeur et lanceur sont bloqués on priorise lanceur
          positionNote = PositionNote.AUCUNE;
        }
        break;

      case LANCEUR:
        if (!isNoteDansLanceur()) {// On valide si la note est sortie du lanceur
          if (isNoteDansGobeur()) {// On valide ou est rendu la note
            positionNote = PositionNote.GOBEUR;
          } else {
            positionNote = PositionNote.AUCUNE;
          }
        }
        break;

      default:// Si il y a un probleme, on remet la note nulle part et on recommence
        positionNote = PositionNote.AUCUNE;
        break;
    }

    SmartDashboard.putString("Mode", mode.toString());
    SmartDashboard.putString("Position Note", positionNote.toString());

    // SmartDashboard.putBoolean("Capteur Lanceur", isNoteDansLanceur());
    // SmartDashboard.putBoolean("Capteur Gobeur", isNoteDansGobeur());

  }

  ////////Mode
  public Mode getMode() {
    return mode;
  }

  public void setModeSpeaker() {
    mode = Mode.SPEAKER;
  }

  public void setModeAmpli() {
    mode = Mode.AMPLI;
  }

  public void setModeGrimpeur() {
    mode = Mode.GRIMPEUR;
  }

  public PositionNote getPositionNote() {
    return positionNote;
  }

  ///////Position Note et capteurs
  public boolean isNoteDansLanceur() {// Idéalement, les Commandes ne parleraient pas aux capteurs, mais seulement à PositionNote.
    return !capteurLanceur.get(); 

  }

  public boolean isNoteDansGobeur() {// Idem
    return !capteurGobeur.get();
  }

  /////////DEL
  public void closeDel() {
    for (var i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setRGB(i, 0, 0, 0);
    }
    del.setData(delBuffer);

  }

  public void setCouleur(Color color) {
    for (int i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setLED(i, color);
    }
    del.setData(delBuffer);

  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < delBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / delBuffer.getLength())) % 180;
      // Set the value
      delBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;

    del.setData(delBuffer);
  }

}
