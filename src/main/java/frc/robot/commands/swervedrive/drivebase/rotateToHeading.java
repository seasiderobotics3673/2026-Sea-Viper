// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GeneralMethods;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rotateToHeading extends Command {

  private SwerveSubsystem drivebase;
  private GeneralMethods utils;

  private Rotation2d destinationHeading;
  private Rotation2d currentHeading;

  private Rotation2d deltaHeading;

  private Rotation2d rotationTolerance;
  private double speedTolerance;

  private double rotationSpeed;

  private int counter;

  private boolean isFinishedFlag;

  /** Creates a new rotateToHeading. */
  public rotateToHeading(SwerveSubsystem drivebase, Rotation2d destinationHeading) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.destinationHeading = destinationHeading;
    this.drivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    utils = new GeneralMethods();

    new Rotation2d();
    rotationTolerance = Rotation2d.fromDegrees(0.25);

    speedTolerance = 0.15;

    isFinishedFlag = false;

    //To pass initial tolerance comparison
    rotationSpeed = speedTolerance + 0.02;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = drivebase.getHeading();

    deltaHeading = destinationHeading.minus(currentHeading);
    counter++;

    if (utils.compareToTolerance(
      //(destinationHeading.getDegrees() - rotationTolerance.getDegrees()), 
      (-speedTolerance),
      (speedTolerance), 
      rotationSpeed, 
      true)) {
        if (deltaHeading.getDegrees() > 0.0) {
          //rotationSpeed = 0.5;
          rotationSpeed = drivebase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), Constants.MAX_ANGULAR_SPEED*0.8, 15);
        } else {
          //rotationSpeed = -0.5;
          rotationSpeed = drivebase.scaleSpeed(currentHeading.getDegrees(), destinationHeading.getDegrees(), -Constants.MAX_ANGULAR_SPEED*0.8, 15);
        }
    } else {
      rotationSpeed = 0.0;
    }

    if (counter >= 10) {
      System.out.println("Rotation Speed: " + rotationSpeed);
      counter = 0;
    }

    drivebase.drive(new ChassisSpeeds(0,0,rotationSpeed));

    if (rotationSpeed == 0.0) {
      isFinishedFlag = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Heading: " + drivebase.getHeading());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedFlag;
  }
}
