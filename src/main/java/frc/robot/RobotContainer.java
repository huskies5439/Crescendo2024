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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // The robot's subsystems
  private final Superstructure superstructure = new Superstructure();
  private final BasePilotable basePilotable = new BasePilotable();
  private final Gobeur gobeur = new Gobeur();
  private final Lanceur lanceur = new Lanceur();
  private final Limelight limelight = new Limelight();
  private final Echelle echelle = new Echelle();
  private final Grimpeur grimpeurGauche = new Grimpeur(1, false, "gauche");
  private final Grimpeur grimpeurDroit = new Grimpeur(2, true, "droit");

  // The driver's controller
  CommandXboxController manette = new CommandXboxController(0);
  private final SendableChooser<Command> chooser;

  Trigger grimpeurTrigger = new Trigger(()-> {return superstructure.getMode() == Mode.GRIMPEUR;});
  Trigger pasGrimpeurTrigger= grimpeurTrigger.negate(); 

  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    //On ne peut pas changer les noms des commandes après avoir créer un Auto dans PathPlanner
    //Les onlyIf sont-ils nécessaires en auto ??
    NamedCommands.registerCommand("gober", new Gober(gobeur,superstructure));
    NamedCommands.registerCommand("lancerSpeaker", new LancerSpeaker(gobeur, lanceur)
                                        /*.onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;})*/.withTimeout(2));//Ajuster le timer selon le délai
    NamedCommands.registerCommand("lancerAmpli", new LancerAmpli(echelle, lanceur, gobeur)
                                        /*.onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.LANCEUR;})*/
                                        .finallyDo(superstructure::setModeSpeaker).withTimeout(2)); //Ajuster le timer selon le délai
    NamedCommands.registerCommand("preparerAmpli", new PreparerAmpli(gobeur, lanceur, superstructure)
                                        /*.onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;})*/);
    

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("trajets", chooser);

    // Commandes par défaut
    basePilotable.setDefaultCommand(
        Commands.run(
            () -> basePilotable.conduire(
                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                true, true),
            basePilotable));
        
    limelight.setDefaultCommand(new UpdatePosition(basePilotable, limelight));
    superstructure.setDefaultCommand(new GestionDEL(superstructure)); 
    echelle.setDefaultCommand(new PositionDefautEchelle(echelle, superstructure));
  
}


  private void configureButtonBindings() {
    
    /////////////Auto Centrer
    manette.a().and(pasGrimpeurTrigger).whileTrue(new ConditionalCommand(
                                                  basePilotable.followPath(true), // Centrer speaker
                                                  basePilotable.followPath(false), // Centrer ampli
                                                  ()->{return superstructure.getMode() == Mode.SPEAKER;})); // Selon mode robot

    manette.start().onTrue(new PreparationPit(echelle, grimpeurGauche, grimpeurDroit));

    //Gobeur
    manette.leftBumper().and(pasGrimpeurTrigger).whileTrue(new Gober(gobeur,superstructure));



    ////////GRIMPEUR
    manette.y().toggleOnTrue(new ToggleModeGrimpeur(superstructure));//ajouter only if 30 sec

    //Position automatique du grimpeur quand on change de mode
    grimpeurTrigger.onTrue(grimpeurDroit.monter().alongWith(grimpeurGauche.monter()))
                   .onFalse(grimpeurDroit.descendre().alongWith(grimpeurGauche.descendre()));

    //Monter et descendre le grimpeur gauche
    manette.leftBumper().and(grimpeurTrigger).whileTrue(grimpeurGauche.monter());
    manette.leftTrigger().whileTrue(grimpeurGauche.descendre());

    //Monter et descendre le grimpeur droit
    manette.rightBumper().and(grimpeurTrigger).whileTrue(grimpeurDroit.monter());
    manette.rightTrigger().whileTrue(grimpeurDroit.descendre());
    
    //Lanceur                                                                             
    manette.x().onTrue( new PreparerAmpli(gobeur, lanceur, superstructure)
                        .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}));
    
    //Lancer au speaker ou l'ampli selon le mode actuel
     manette.rightBumper().and(pasGrimpeurTrigger).whileTrue(new ConditionalCommand(//Selon le mode du robot
      
      new LancerSpeaker(gobeur, lanceur)
          .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}) ,

      new LancerAmpli(echelle, lanceur, gobeur)
          .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.LANCEUR;})
          .finallyDo(superstructure::setModeSpeaker) ,

      () -> {return superstructure.getMode() == Mode.SPEAKER;}));

    

   
      
    //Commandes pour valider les systèmes
    //Up, Down -> grimpeur
    manette.povUp().whileTrue(Commands.startEnd(()->grimpeurGauche.setVoltage(3), grimpeurGauche::stop, grimpeurGauche)
                      .alongWith(Commands.startEnd(()->grimpeurDroit.setVoltage(3), grimpeurDroit::stop, grimpeurDroit)));

    manette.povDown().whileTrue(Commands.startEnd(()->grimpeurGauche.setVoltage(-3), grimpeurGauche::stop, grimpeurGauche)
                      .alongWith(Commands.startEnd(()->grimpeurDroit.setVoltage(-3), grimpeurDroit::stop, grimpeurDroit)));

    //Gauche, Droite -> échelle
    manette.povRight().onTrue(echelle.setPIDCommand(0.2));
    manette.povLeft().onTrue(echelle.setPIDCommand(0.0));

    //B -> Lanceur de base
    manette.b().toggleOnTrue(lanceur.setPIDCommand(4));
    // manette.b().toggleOnTrue(lanceur.commandeVoltageSimple(4));

  }


  public Command getAutonomousCommand() {
    return chooser.getSelected();
    
  }

}
