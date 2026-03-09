// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GeneralMethods;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveNearTargetPoint extends Command {

  private SwerveSubsystem drivebase;
  private Vision vision;
  private Cameras cameraEnum;

  private double destDistance;
  private Translation2d point;

  private double xTolerance;
  private double yTolerance;

  private int directionInverse;

  private ArrayList<Translation2d> previousTranslations;

  private boolean isFinishedFlag;

  private double driveSpeedX;
  private ChassisSpeeds driveSpeeds;

  

  /** Creates a new moveNearTargetPoint.
   * Presumes that the bot is already facing the point, 
   */
  public moveNearTargetPoint(SwerveSubsystem drivebase, Vision vision, Cameras cameraEnum, double destDistance) {
    this.destDistance = destDistance;
    this.drivebase = drivebase;
    this.vision = vision;
    this.cameraEnum = cameraEnum;
    
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yTolerance = 0.05;
    xTolerance = 0.03;

    drivebase.getSwerveDrive().setHeadingCorrection(true);

    previousTranslations = new ArrayList<Translation2d>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    point = vision.getHUBCenterPoint(cameraEnum, drivebase);

    if (!point.equals(new Translation2d())) {
      previousTranslations.add(point);
    } else {
        if (previousTranslations.size() == 0) {
          DriverStation.reportWarning("Attempted to get member from previousTransforms while empty", false);
        } else {
          DriverStation.reportWarning("apriltagTransform3d returned empty", false);
          point = previousTranslations.get(previousTranslations.size() - 1);
        }
    }

    //If we see no Apriltags and havent gotten any previous points yet.
    if (point.equals(new Translation2d()) && previousTranslations.size() == 0) {
      isFinishedFlag = true;
    }

    if (previousTranslations.size() >= 5) {
      previousTranslations.subList(1, previousTranslations.size()).clear();
    }

    if (destDistance > point.getX()) {
      directionInverse = -1;
    } else {
      //If the target distance is closer to the Apriltag than we are currently
      directionInverse = 1;
    }

    
    if (GeneralMethods.compareToTolerance((destDistance - xTolerance), (destDistance + xTolerance), point.getX(), true)) {

    if (point.getX() >= 0) {
        //driveSpeedX = Constants.MAX_SPEED/4 * directionInverse;
        driveSpeedX = drivebase.scaleSpeed(point.getX(), destDistance, Constants.MAX_SPEED*0.5, 1) * directionInverse;
      } else { 
        //driveSpeedX = -Constants.MAX_SPEED/4 * directionInverse;
        driveSpeedX = -drivebase.scaleSpeed(point.getX(), destDistance, Constants.MAX_SPEED*0.5, 1) * directionInverse;
      }

    } else {
      driveSpeedX = 0.0;
    }

    if (Double.compare(driveSpeedX, 0.0) == 0 &&  
        !point.equals(new Translation2d())) { isFinishedFlag = true; }

    driveSpeeds = new ChassisSpeeds(driveSpeedX, 0, 0);

    drivebase.drive(driveSpeeds);

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
