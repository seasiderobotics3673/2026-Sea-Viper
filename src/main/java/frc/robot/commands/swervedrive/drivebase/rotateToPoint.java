// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GeneralMethods;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rotateToPoint extends Command {

  private SwerveSubsystem drivebase;

  private Translation2d point;

  private Rotation2d destinationHeading;
  private Rotation2d currentHeading;

  private Rotation2d deltaHeading;

  private double speedTolerance;

  private double rotationSpeed;

  private boolean isFinishedFlag;

  /** Creates a new rotateToPoint. */
  public rotateToPoint(SwerveSubsystem drivebase, Translation2d point) {
    this.point = point;
    this.drivebase = drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    destinationHeading = GeneralMethods.calculateAngleToPoint(point);

    speedTolerance = 0.15;

    isFinishedFlag = false;

    rotationSpeed = speedTolerance + 0.02;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = drivebase.getHeading();
    deltaHeading = destinationHeading.minus(currentHeading);

    if (GeneralMethods.compareToTolerance(
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

    drivebase.drive(new ChassisSpeeds(0,0,rotationSpeed));

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
    return isFinishedFlag;
  }
}
