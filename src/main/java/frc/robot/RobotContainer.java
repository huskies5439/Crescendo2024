// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commmands.GestionDEL;
import frc.robot.commmands.Gober;
import frc.robot.commmands.Homing;
import frc.robot.commmands.LancerAmpli;
import frc.robot.commmands.LancerSpeaker;
import frc.robot.commmands.PreparerAmpli;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


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


  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("gober", new Gober(gobeur,superstructure));
    NamedCommands.registerCommand("lancer", new WaitCommand(2));
    NamedCommands.registerCommand("scorerAmpli", new WaitCommand(2));

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
  
  //À ajouter au code quand on est confiant du comportement de l'échelle
  //echelle.setDefaultCommand(new ConditionalCommand(echelle.setPIDCommand(0.2), echelle.setPIDCommand(0), () -> {return superstructure.getMode() == Mode.GRIMPER;}));

}


  private void configureButtonBindings() {

    manette.a().whileTrue(Commands.run(basePilotable::setX, basePilotable));
    manette.leftBumper().whileTrue(new Gober(gobeur,superstructure));
    manette.start().onTrue(new Homing(echelle));

    manette.leftTrigger().whileTrue(grimpeurGauche.descendre());
    manette.rightTrigger().whileTrue(grimpeurDroit.descendre());
    //À changer pour mode Grimper
    manette.y().onTrue(grimpeurGauche.monter().alongWith(grimpeurDroit.monter())); // et éventuellement.alongWith(new
                                                                                   // PIDEchelle(0.2)).....
                                                                                   // et éventuellement il faut passer en mode grimpeur

                                                                                  
    manette.x().onTrue(new PreparerAmpli(echelle, gobeur, lanceur, superstructure)//Préparer ampli ne fonctionne pas tant qu´il n´y a pas de note dans le gobeur
              .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}));
    
    manette.rightBumper().whileTrue(new ConditionalCommand(//Selon le mode du robot
      new LancerSpeaker(echelle, gobeur, lanceur, superstructure)
        .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.GOBEUR;}),

      new LancerAmpli(echelle, lanceur, superstructure)
        .onlyIf(() -> {return superstructure.getPositionNote() == PositionNote.LANCEUR;}),

      () -> {return superstructure.getMode() == Mode.SPEAKER;}));
    
      
    //Commandes pour valider les systèmes
    //Up, Down -> grimpeur
    manette.povUp().whileTrue(Commands.startEnd(()->grimpeurGauche.setVoltage(true), grimpeurGauche::stop, grimpeurGauche)
                      .alongWith(Commands.startEnd(()->grimpeurDroit.setVoltage(true), grimpeurDroit::stop, grimpeurDroit)));

    manette.povDown().whileTrue(Commands.startEnd(()->grimpeurGauche.setVoltage(false), grimpeurGauche::stop, grimpeurGauche)
                      .alongWith(Commands.startEnd(()->grimpeurDroit.setVoltage(false), grimpeurDroit::stop, grimpeurDroit)));

    //Gauche, Droite -> échelle
    manette.povRight().whileTrue(Commands.startEnd(()->echelle.setVoltage(3), echelle::stop, echelle));
    manette.povLeft().whileTrue(Commands.startEnd(()->echelle.setVoltage(-3), echelle::stop, echelle));

    //B -> Lanceur de base
    manette.b().toggleOnTrue(lanceur.setPIDCommand(10));
    //manette.b().toggleOnTrue(lanceur.commandeVoltageSimple(3));

  }


  public Command getAutonomousCommand() {
    return chooser.getSelected();
    
  }

}
