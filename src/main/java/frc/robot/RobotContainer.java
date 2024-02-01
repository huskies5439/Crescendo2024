// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commmands.Ajuster;
import frc.robot.commmands.Gober;
import frc.robot.commmands.Grimper;
import frc.robot.commmands.UpdatePosition;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Grimpeur;
import frc.robot.subsystems.GrimpeurV2;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {
  // The robot's subsystems
  private final BasePilotable basePilotable = new BasePilotable();
  private final Gobeur gobeur = new Gobeur();
  private final Lanceur lanceur = new Lanceur();
  private final Limelight limelight = new Limelight();
  private final SendableChooser<Command> chooser;
  private final Grimpeur grimpeur = new Grimpeur();


  //////Suggestion V2 pour team grimpeur
  private final GrimpeurV2 grimpeurGauche = new GrimpeurV2(1, false, "gauche");
  //On crée un autre GrimpeurV2 qui est grimpeurDroit


  // The driver's controller
  CommandXboxController manette = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    NamedCommands.registerCommand("gober", new Gober(gobeur));
    NamedCommands.registerCommand("lancer", new WaitCommand(2));
    NamedCommands.registerCommand("scorerAmpli", new WaitCommand(2));
    
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("trajets",chooser);


    // Configure default commands
    basePilotable.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> basePilotable.conduire(
                  manette.getLeftY(),manette.getLeftX(),manette.getRightX(),
                  true, true),
                  basePilotable)); 
    limelight.setDefaultCommand(new UpdatePosition(basePilotable,limelight));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   
    manette.x().whileTrue(Commands.run(basePilotable::setX, basePilotable));
    manette.start().onTrue(Commands.runOnce(basePilotable::resetGyro));
    manette.leftBumper().toggleOnTrue(new Gober(gobeur));
    manette.y().toggleOnTrue(Commands.startEnd(lanceur::setVoltageShuffleboard, lanceur::stop, lanceur));
    manette.rightTrigger().whileTrue(new Ajuster(grimpeur,2));
    manette.leftTrigger().whileTrue(new Ajuster(grimpeur,1));
    manette.b().onTrue(new Grimper(grimpeur));



    //Proposition V1 pour team grimpeur
    manette.leftTrigger().whileTrue(grimpeur.descendreGauche());//whileTrue fait qu'on va canceller la commande en lâchant
    manette.b().onTrue(grimpeur.monterGauche());/* .alongWith(équivalent pour la droite); */


    //Proposition V2 pour team grimpeur
    manette.leftTrigger().whileTrue(grimpeurGauche.descendre());
    //manette.rightTrigger().whileTrue(grimpeurDroit.descendre());
    manette.b().onTrue(grimpeurGauche.monter());//.alongWith(grimpeurDroit.monter())// et éventuellement.alongWith(new PIDEchelle(0.2)).....

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return chooser.getSelected();


  }


  
}
