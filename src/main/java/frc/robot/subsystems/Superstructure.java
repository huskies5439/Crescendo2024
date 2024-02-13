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

  private final DigitalInput capteurGobeur = new DigitalInput(0); // Émeteur branché sur le 1
  private final DigitalInput capteurLanceur = new DigitalInput(2); // Émetteur sera branché sur le 3

  private final AddressableLED del = new AddressableLED(9); // Port PWM, pas DIO
  private final AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(8); // LE nombre de sections de DEL ici 3
                                                                              // DEL/Section

  /** Creates a new Superstructure. */
  public Superstructure() {
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
    closeDel();
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
        if (!isNoteDansGobeur()) {// On valide si il n'y a plus de note dans le gobeur
          if (isNoteDansLanceur()) {// On valide ou est rendu la note
            positionNote = PositionNote.LANCEUR;
          } else {
            positionNote = PositionNote.AUCUNE;
          }
        }
        break;
      case LANCEUR://Possible de mettre la logique de DetecterNoteLanceur ici ?????????
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

    SmartDashboard.putBoolean("Capteur Lanceur", isNoteDansLanceur());
    SmartDashboard.putBoolean("Capteur Gobeur", isNoteDansGobeur());

  }

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

  public boolean isNoteDansLanceur() {//Idéalement, les Commandes ne parleraient pas aux capteurs, mais seulement à Position.Note
    return !capteurLanceur.get(); // verifier s'il faut inverser ( veut qu'il dise true quand on a la note )

  }

  public boolean isNoteDansGobeur() {//Idem
    return !capteurGobeur.get(); // verifier s'il faut inverser ( veut qu'il dise true quand on a la note )
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
