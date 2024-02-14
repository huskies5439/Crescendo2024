// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BasePilotable extends SubsystemBase {
  // Créer les moteurs swerves
  private final MAXSwerveModule avantGauche = new MAXSwerveModule(1,2,-90);

  private final MAXSwerveModule avantDroite = new MAXSwerveModule(3,4,0);

  private final MAXSwerveModule arriereGauche = new MAXSwerveModule(5,6,180);

  private final MAXSwerveModule arriereDroite = new MAXSwerveModule(7,8,90);

  // Le gyroscope
  private final Pigeon2 gyro = new Pigeon2(1);

  // Slew rate filter variables for controlling lateral acceleration (Code de REV)
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //Initialisation PoseEstimator
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          avantGauche.getPosition(),
          avantDroite.getPosition(),
          arriereGauche.getPosition(),
          arriereDroite.getPosition()
      },
      new Pose2d());

  public BasePilotable() {
    //Reset initial
    resetGyro(); 
    resetEncoders();
    resetOdometry(new Pose2d());

    //Initialisation de PathPlanner (selon leur getting started)
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeed,
      this::conduireChassis,
      DriveConstants.kPathFollowerConfig,
       () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
    );
  }

  @Override
  public void periodic() {
    // Update du Pose Estimator
    poseEstimator.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            avantGauche.getPosition(),
            avantDroite.getPosition(),
            arriereGauche.getPosition(),
            arriereDroite.getPosition()
        });

        
        // SmartDashboard.putNumber("Gyro", getAngle());

        // SmartDashboard.putNumber("Pose Estimator X",getPose().getX());
        // SmartDashboard.putNumber("Pose Estimator Y",getPose().getY());
        // SmartDashboard.putNumber("Pose Estimator Rotation",getPose().getRotation().getDegrees());

        // SmartDashboard.putNumber("Chassis Speed VX", getChassisSpeed().vxMetersPerSecond);
        // SmartDashboard.putNumber("Chassis Speed VY", getChassisSpeed().vyMetersPerSecond);
        // SmartDashboard.putNumber("Chassis Speed Omega", Math.toDegrees(getChassisSpeed().omegaRadiansPerSecond));
        
  }



  /////////MÉTHODE DONNANT DES CONSIGNES À CHAQUE MODULE
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.maxVitesseModule);
    avantGauche.setDesiredState(desiredStates[0]);
    avantDroite.setDesiredState(desiredStates[1]);
    arriereGauche.setDesiredState(desiredStates[2]);
    arriereDroite.setDesiredState(desiredStates[3]);
  }



  ////////TÉLÉOP
  public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded; 
    double ySpeedCommanded;

    double deadband = 0.05;
    //appliquer une deadband sur les joysticks et corriger la direction
    xSpeed = -MathUtil.applyDeadband(xSpeed, deadband);
    ySpeed = -MathUtil.applyDeadband(ySpeed, deadband);
    rot = -MathUtil.applyDeadband(rot, deadband); 



    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.maxVitesseLineaire;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.maxVitesseLineaire;
    double rotDelivered = m_currentRotation * DriveConstants.maxVitesseRotation;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    setModuleStates(swerveModuleStates);
  }


  public void stop() {
    conduire(0,0,0,false,false);

  }



   //Sets the wheels into an X formation to prevent movement.
  public void setX() {
    avantGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    avantDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }



  /////////Pose estimator
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose2d position, double delaiLimelight) {
    poseEstimator.addVisionMeasurement(position, Timer.getFPGATimestamp() - delaiLimelight);
  }

  public void resetOdometry(Pose2d pose) {//pose est à la pose où reset l'odométrie du robot
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            avantGauche.getPosition(),
            avantDroite.getPosition(),
            arriereGauche.getPosition(),
            arriereDroite.getPosition()
        },
        pose);
  }


  //////////////Encodeurs
  //Pas besoin de méthode pour obtenir la position des encodeurs, tout ça passe directement pas la pose2D du robot
  public void resetEncoders() {
    avantGauche.resetEncoders();
    arriereGauche.resetEncoders();
    avantDroite.resetEncoders();
    arriereDroite.resetEncoders();
  }



  ///////////////GYRO
  public double getAngle() {
    return -gyro.getAngle();
  } 

    public void resetGyro() {
    gyro.setYaw(0);
  }



  /////////Méthodes pour PathPlanner. Ce sont deux méthodes réfléchies en terme du Chassis et Field Relative
/**
   * Returns the chassis speed relative to the field XY and angle from each module angle and speed
   * @return the ChassisSpeed
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(avantDroite.getState(), avantGauche.getState(),arriereDroite.getState(), arriereGauche.getState());
  }

  /**
   * Method to drive the robot from field relative speeds
   * @param chassisSpeeds XY and omege speeds the robot should be going
   */
  public void conduireChassis(ChassisSpeeds chassisSpeeds) {
    //Ramene la vitesse continue en valeur à chaque de 20 ms (fréquence des itérations du roborio = 50 Hz)
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(swerveModuleStates); 
   }
  
  
}
