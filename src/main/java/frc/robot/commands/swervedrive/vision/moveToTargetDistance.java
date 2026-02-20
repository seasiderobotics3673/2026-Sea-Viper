// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GeneralMethods;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveToTargetDistance extends Command {

  private SwerveSubsystem drivebase;
  private Vision vision;

  private double destDistance;
  private double desiredHeading;
  private Transform3d apriltagTransform3d;
  private int fiducialId;
  private boolean isRegardingSpecificID;
  private Cameras cameraEnum = Cameras.OFFSET_CAM;
  private GeneralMethods generalMethods;

  private ChassisSpeeds driveSpeeds;
  private double driveSpeedX;
  private double driveSpeedY;
  private double rotationSpeed;

  private double yTolerance;
  private double xTolerance;
  private double rotationTolerance;
  private int directionInverse;

  private Rotation2d currentHeading;

  private ArrayList<Transform3d> previousTransforms;

  private Translation3d offsetPointBot;
  private Translation3d offsetPointTag;

  private int counter;
  private boolean isFinishedFlag;


  /** Creates a new moveToTargetDistance. */
  public moveToTargetDistance(double destDistance, SwerveSubsystem drivebase, Vision vision, int fiducialId, Translation3d offsetPointBot, Translation3d offsetPointTag, double desiredHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destDistance = destDistance;
    this.drivebase = drivebase;
    this.vision = vision;
    this.fiducialId = fiducialId;
    this.offsetPointBot = offsetPointBot;
    this.offsetPointTag = offsetPointTag;
    this.desiredHeading = desiredHeading;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    yTolerance = 0.05;
    xTolerance = 0.03;
    rotationTolerance = 1.25;

    directionInverse = 1;

    isFinishedFlag = false;

    previousTransforms = new ArrayList<Transform3d>(10);

    generalMethods = new GeneralMethods();

    if (fiducialId <= 0 || fiducialId > Constants.APRILTAG_HEIGHTS.length) {
      isRegardingSpecificID = false;
    } else {
      isRegardingSpecificID = true;
    }

    drivebase.getSwerveDrive().setHeadingCorrection(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    //apriltagTrans2d = vision.getTargetPos(cameraEnum, isRegardingSpecificID, fiducialId);
    apriltagTransform3d = vision.getTargetTransformOffset(cameraEnum, offsetPointBot, isRegardingSpecificID, fiducialId);

    if (!apriltagTransform3d.equals(new Transform3d())) {
      previousTransforms.add(apriltagTransform3d);
    } else {
        if (previousTransforms.size() == 0) {
          DriverStation.reportWarning("Attempted to get member from previousTransforms while empty", false);
        } else {
          DriverStation.reportWarning("apriltagTransform3d returned empty", false);
          apriltagTransform3d = previousTransforms.get(previousTransforms.size() - 1);
        }
    }

    if (apriltagTransform3d.equals(new Transform3d()) && previousTransforms.size() == 0) {
      isFinishedFlag = true;
    }

    if (previousTransforms.size() >= 5) {
      previousTransforms.subList(1, previousTransforms.size()).clear();
    }

    counter++;

    if (counter >= 10) {
      //System.out.println("Estimated X: " + apriltagTransform3d.getX());
      //System.out.println("Estimated Y: " + apriltagTransform3d.getY());
      //System.out.println("Most Recent Transform: "+ previousTransforms.get(previousTransforms.size() - 1));
      counter = 0;
    }

    apriltagTransform3d = new Transform3d(apriltagTransform3d.getTranslation().plus(offsetPointTag), apriltagTransform3d.getRotation());

    //If our target distance is farther away from the Apriltag than we are currently
    if (destDistance > apriltagTransform3d.getX()) {
      directionInverse = -1;
    } else {
      //If the target distance is closer to the Apriltag than we are currently
      directionInverse = 1;
    }

    currentHeading = drivebase.getHeading();

    if (GeneralMethods.compareToTolerance((destDistance - xTolerance), (destDistance + xTolerance), apriltagTransform3d.getX(), true)) {

      if (apriltagTransform3d.getX() >= 0) {
        //driveSpeedX = Constants.MAX_SPEED/4 * directionInverse;
        driveSpeedX = drivebase.scaleSpeed(apriltagTransform3d.getX(), destDistance, Constants.MAX_SPEED*0.5, 1) * directionInverse;
      } else { 
        //driveSpeedX = -Constants.MAX_SPEED/4 * directionInverse;
        driveSpeedX = -drivebase.scaleSpeed(apriltagTransform3d.getX(), destDistance, Constants.MAX_SPEED*0.5, 1) * directionInverse;
      }

    } else {
      driveSpeedX = 0.0;
    }

    if (GeneralMethods.compareToTolerance(-yTolerance, yTolerance, apriltagTransform3d.getY(), true)) {

      if (apriltagTransform3d.getY() >= 0) {
        //driveSpeedY = Constants.MAX_SPEED*0.2;
        driveSpeedY = drivebase.scaleSpeed(apriltagTransform3d.getY(), 0, Constants.MAX_SPEED*0.4, 0.5);
      } else {
        //driveSpeedY = -Constants.MAX_SPEED*0.2
        driveSpeedY = -drivebase.scaleSpeed(apriltagTransform3d.getY(), 0, Constants.MAX_SPEED*0.4, 0.5);
      }

    } else {
      driveSpeedY = 0.0;
    }

    if (GeneralMethods.compareToTolerance(
      (desiredHeading - rotationTolerance), 
      (desiredHeading + rotationTolerance), 
      currentHeading.getDegrees(), 
      true)) {
        if (currentHeading.getDegrees() >= 0) {
          rotationSpeed = -Constants.MAX_ANGULAR_SPEED*0.2;
        } else {
          rotationSpeed = Constants.MAX_ANGULAR_SPEED*0.2;
        }
      } else {
        rotationSpeed = 0.0;
      }

    if (Double.compare(driveSpeedX, 0.0) == 0 && 
        Double.compare(driveSpeedY, 0.0) == 0 && 
        !apriltagTransform3d.equals(new Transform3d())) { isFinishedFlag = true; }

    //driveSpeeds = new ChassisSpeeds(driveSpeedX, driveSpeedY, rotationSpeed);
    driveSpeeds = new ChassisSpeeds(driveSpeedX, driveSpeedY, 0);

    drivebase.drive(driveSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Ended");
    System.out.println("");
    drivebase.getSwerveDrive().setHeadingCorrection(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedFlag;
  }
}
