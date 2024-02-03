// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  public enum Mode {
    SPEAKER,
    AMPLI
  }

  public enum PositionNote {
    AUCUNE,
    GOBEUR,
    LANCEUR
  }

  private Mode mode = Mode.SPEAKER;

  private PositionNote positionNote = PositionNote.AUCUNE;

  private final DigitalInput capteurLanceur = new DigitalInput(2);
  private final DigitalInput capteurGobeur = new DigitalInput(0); // Émeteur branché sur le 1
  private AddressableLED del = new AddressableLED(9);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(8); // LE nombre de sections de DEL ici 3
                                                                        // DEL/Section

  /** Creates a new Superstructure. */
  public Superstructure() {
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
    setCouleur(Color.kOrange);
  }

  @Override
  public void periodic() {
    switch (positionNote) {
      case AUCUNE:
        if (isNoteDansGobeur()) {
          positionNote = PositionNote.GOBEUR;
        } else if (isNoteDansLanceur()) {
          positionNote = PositionNote.LANCEUR;
        }
        break;
      case GOBEUR:
        if (!isNoteDansGobeur()) {
          if (isNoteDansLanceur()) {
            positionNote = PositionNote.LANCEUR;
          } else {
            positionNote = PositionNote.AUCUNE;
          }
        }
        break;
      case LANCEUR:
        if (!isNoteDansLanceur()) {
          if (isNoteDansGobeur()) {
            positionNote = PositionNote.GOBEUR;
          } else {
            positionNote = PositionNote.AUCUNE;
          }
        }
        break;
      default:
        positionNote = PositionNote.AUCUNE;
        break;
    }
  }

  public Mode getMode() {
    return mode;
  }

  public PositionNote getPositionNote() {
    return positionNote;
  }

  public boolean isNoteDansLanceur() {
    return capteurLanceur.get(); // verifier s'il faut inverser ( veut qu'il dise true quand on a la note )

  }

  public boolean isNoteDansGobeur() {
    return capteurGobeur.get();
  }

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
}
