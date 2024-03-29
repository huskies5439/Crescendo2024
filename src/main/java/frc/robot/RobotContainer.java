// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commmands.GestionDEL;
import frc.robot.commmands.Gober;
import frc.robot.commmands.PreparationPit;
import frc.robot.commmands.LancerAmpli;
import frc.robot.commmands.LancerSpeaker;
import frc.robot.commmands.PositionDefautEchelle;
import frc.robot.commmands.PreparerAmpli;
import frc.robot.commmands.ToggleModeGrimpeur;
import frc.robot.commmands.UpdatePosition;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Echelle;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Grimpeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;
import frc.robot.subsystems.Superstructure.PositionNote;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ChiffreMagique.*;//Pour alléger les commandes

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  private final Superstructure superstructure = new Superstructure();
  private final BasePilotable basePilotable = new BasePilotable();
  private final Gobeur gobeur = new Gobeur();
  private final Lanceur lanceur = new Lanceur();
  private final Limelight limelight = new Limelight();
  private final Echelle echelle = new Echelle();
  private final Grimpeur grimpeurGauche = new Grimpeur(1, false, "gauche");
  private final Grimpeur grimpeurDroit = new Grimpeur(2, true, "droit");

  
  CommandXboxController manette = new CommandXboxController(0);

  private final SendableChooser<Command> chooser;

  //Grimpeur arbitraire pour le monde Grimpeur
  Trigger grimpeurTrigger = new Trigger(()-> {return superstructure.getMode() == Mode.GRIMPEUR;});
  Trigger pasGrimpeurTrigger= grimpeurTrigger.negate(); //Permet d'alléger l'assignation des boutons

  public RobotContainer() {
    configureButtonBindings();

   //Créer les commandes pour PathPlanner
    NamedCommands.registerCommand("gober", new Gober(gobeur,superstructure));
    NamedCommands.registerCommand("lancerSpeaker", new LancerSpeaker(gobeur, lanceur)
                                       .withTimeout(1.5));

    NamedCommands.registerCommand("lancerAmpli", new LancerAmpli(echelle, lanceur, gobeur)
                                        .finallyDo(superstructure::setModeSpeaker)
                                        .withTimeout(1.5)); 

    NamedCommands.registerCommand("preparerAmpli", new PreparerAmpli(gobeur, lanceur, superstructure));
    NamedCommands.registerCommand("descendreEchelle",echelle.setPIDCommand(0.0).until(echelle::isPositionDepart));
    NamedCommands.registerCommand("preparerSpeaker", lanceur.setPIDCommandSansFin(vitesseLancerSpeaker));

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("trajet", chooser);

    // Commandes par défaut
    basePilotable.setDefaultCommand(
        Commands.run(
            () -> basePilotable.conduire(
                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                true, true, true),
            basePilotable));
        
    limelight.setDefaultCommand(new UpdatePosition(basePilotable, limelight));
    superstructure.setDefaultCommand(new GestionDEL(superstructure)); 
    echelle.setDefaultCommand(new PositionDefautEchelle(echelle, superstructure));

}


  private void configureButtonBindings() {
    
    /////////////Auto Centrer
    manette.a().and(pasGrimpeurTrigger).whileTrue(
      new ConditionalCommand(
          basePilotable.followPath(true).alongWith(lanceur.setPIDCommand(vitesseLancerSpeaker)), // Centrer speaker
          basePilotable.followPath(false), // Centrer ampli
          ()->{return superstructure.getMode() == Mode.SPEAKER;})); // Selon mode robot

    //Gobeur
    manette.leftBumper().and(pasGrimpeurTrigger).toggleOnTrue(new Gober(gobeur,superstructure));


    
    //////////////////////////Lanceur 
    //Préparer Ampli                                                                            
    manette.x().onTrue( new PreparerAmpli(gobeur, lanceur, superstructure)
                        .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}));
    
    //Lancer au speaker ou l'ampli selon le mode actuel
     manette.rightBumper().and(pasGrimpeurTrigger).whileTrue(new ConditionalCommand(//Selon le mode du robot
      
      new LancerSpeaker(gobeur, lanceur)
         .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}) ,

      new LancerAmpli(echelle, lanceur, gobeur)
          //.onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.LANCEUR;})
          .finallyDo(superstructure::setModeSpeaker) ,

      () -> {return superstructure.getMode() == Mode.SPEAKER;}));

     
    //monter l'échelle si la note reste pris sur l'ampli
    manette.rightTrigger().and(pasGrimpeurTrigger).whileTrue(echelle.setPIDCommand(HauteurAmpli)); 


    ////////////////////////////GRIMPEUR
    manette.y().toggleOnTrue(new ToggleModeGrimpeur(superstructure));//ajouter only if 30 sec ?

    //Position automatique du grimpeur quand on change de mode
    grimpeurTrigger.onTrue(grimpeurDroit.monterCommand(true).alongWith(grimpeurGauche.monterCommand(true)))
                   .onFalse(grimpeurDroit.descendreCommand(false).alongWith(grimpeurGauche.descendreCommand(false)));

    //Monter et descendre le grimpeur gauche
    manette.leftBumper().and(grimpeurTrigger).whileTrue(grimpeurGauche.monterCommand(true));
    manette.leftTrigger().and(grimpeurTrigger).whileTrue(grimpeurGauche.descendreCommand(true));

    //Monter et descendre le grimpeur droit
    manette.rightBumper().and(grimpeurTrigger).whileTrue(grimpeurDroit.monterCommand(true));
    manette.rightTrigger().and(grimpeurTrigger).whileTrue(grimpeurDroit.descendreCommand(true));
      


    //////////Commandes PIT
    //Descendre et monter les grimpeurs dans le pit
    manette.povLeft().whileTrue(grimpeurGauche.descendreCommandSansLimite(false));

    manette.povRight().whileTrue(grimpeurDroit.descendreCommandSansLimite(false));

    manette.povUp().whileTrue(grimpeurGauche.monterCommandSansLimite(false)
                    .alongWith(grimpeurDroit.monterCommandSansLimite(false)));
                      
    //Après avoir descendu les grimpeurs dans le pit, on home l'échelle et reset les encodeurs des grimpeurs                                  
    manette.start().onTrue(new PreparationPit(echelle, grimpeurGauche, grimpeurDroit));


    //////////////PANIC!!
    manette.b().whileTrue(lanceur.setPIDCommand(6).alongWith(gobeur.convoyerCommand()))
        .onFalse(Commands.runOnce(superstructure::setModeSpeaker));

  }


  public Command getAutonomousCommand() {
    return chooser.getSelected();

  }

}
