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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class altDriveCommand extends Command {
  /** Creates a new altDriveCommand. */
  //actualy, make the stuffs you know like dis
  SwerveSubsystem driveBase;
  Vision vision;

  Translation2d point;

  Rotation2d destinationHeading;
  Rotation2d currentHeading;

  Rotation2d deltaHeading;

  double speedTolerance;

  double rotationSpeed;

  boolean isFinishedFlag;


  public altDriveCommand(SwerveSubsystem driveBase, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
    this.driveBase = driveBase;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    destinationHeading = getHUBcenterPoint()

    speedTolerance = 0.15;

    isFinishedFlag = false;

    rotationSpeed = speedTolerance + 0.02;
  }

  // Called every time the scheduler runs while the command is scheduled.
   @Override
  public void execute() {
    currentHeading = driveBase.getHeading();
    deltaHeading = destinationHeading.minus(currentHeading);

    if (GeneralMethods.compareToTolerance(
      (-speedTolerance),
      (speedTolerance), 
      rotationSpeed, 
      true)) {
        if (deltaHeading.getDegrees() > 0.0) {
          //rotationSpeed = 0.5;
          rotationSpeed = driveBase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), Constants.MAX_ANGULAR_SPEED*0.8, 15);
        } else {
          //rotationSpeed = -0.5;
          rotationSpeed = driveBase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), -Constants.MAX_ANGULAR_SPEED*0.8, 15);
        }
    } else {
      rotationSpeed = 0.0;
    }
    /*
    get information from the controller
    use that information in the driveBase.drive
    stop geting the rotationSpeed from the controller, get it from getHUBcenterPoint() -ollys working on that- instead 
    flip the rotation acording to the direction that the hub center is
    */

    driveBase.drive(new ChassisSpeeds(0,0,rotationSpeed));

    if (rotationSpeed == 0.0) {
      isFinishedFlag = true;
    }
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
