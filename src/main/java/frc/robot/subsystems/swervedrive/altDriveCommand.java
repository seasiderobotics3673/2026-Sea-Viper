// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//READ THIS: most of this code is grabed from rotate to point, this just adds movment funcitanality and stuff
package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GeneralMethods;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class altDriveCommand extends Command {
  /** Creates a new altDriveCommand. */
  //actualy, make the stuffs you know like dis
  SwerveSubsystem driveBase;
  Vision vision;

  Cameras camera;

  SwerveInputStream controllerInput;

  double velocityX;
  double velocityY;

  int counter;

  Translation2d point;

  Rotation2d destinationHeading;
  Rotation2d currentHeading;

  Rotation2d deltaHeading;

  double speedTolerance;

  double rotationSpeed;

  boolean isFinishedFlag;


  public altDriveCommand(SwerveSubsystem driveBase, Vision vision, Cameras camera, SwerveInputStream controllerInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.vision = vision;
    this.camera = camera;
    this.controllerInput = controllerInput;
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedTolerance = 0.15;

    isFinishedFlag = false;

    rotationSpeed = speedTolerance + 0.02;

    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
   @Override
  public void execute() {
    counter++;

    currentHeading = driveBase.getHeading();
    if (counter > 50) {
      System.out.println("Angle to HUB: " + vision.getAngleToHUB(camera, driveBase));
      System.out.println("Current Heading: " + driveBase.getHeading());
    } 
    destinationHeading = vision.getAngleToHUB(camera, driveBase);
    deltaHeading = destinationHeading.minus(currentHeading);

    if (!destinationHeading.equals(new Rotation2d())) {
      if (deltaHeading.getDegrees() > 0.0) {
        rotationSpeed = driveBase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), Constants.MAX_ANGULAR_SPEED*0.8, 7.5);
      } else {
        rotationSpeed = driveBase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), -Constants.MAX_ANGULAR_SPEED*0.8, 7.5);
      }
    } else {
      rotationSpeed = 0.0;
    }


    velocityX = controllerInput.get().vxMetersPerSecond;
    velocityY = controllerInput.get().vyMetersPerSecond;

    driveBase.drive(new ChassisSpeeds(velocityX, velocityY, rotationSpeed));
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
