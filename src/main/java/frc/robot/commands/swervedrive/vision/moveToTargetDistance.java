// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private Translation2d apriltagTrans2d;
  private int fiducialId;
  private boolean isRegardingSpecificID;
  private Cameras cameraEnum = Cameras.CENTER_CAM;
  private GeneralMethods generalMethods;

  private ChassisSpeeds driveSpeeds;
  private double driveSpeedX;
  private double driveSpeedY;

  private double yTolerance;
  private double xTolerance;
  private int directionInverse;

  private int counter;
  private boolean isFinishedFlag;


  /** Creates a new moveToTargetDistance. */
  public moveToTargetDistance(double destDistance, SwerveSubsystem drivebase, Vision vision, int fiducialId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destDistance = destDistance;
    this.drivebase = drivebase;
    this.vision = vision;
    this.fiducialId = fiducialId;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    yTolerance = 0.08;
    xTolerance = 0.05;

    directionInverse = 1;

    isFinishedFlag = false;

    generalMethods = new GeneralMethods();

    if (fiducialId <= 0 || fiducialId > Constants.APRILTAG_HEIGHTS.length) {
      isRegardingSpecificID = false;
    } else {
      isRegardingSpecificID = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    apriltagTrans2d = vision.getTargetPos(cameraEnum, isRegardingSpecificID, fiducialId);

    counter++;

    if (counter >= 20) {
      System.out.println("Estimated X: " + apriltagTrans2d.getX());
      System.out.println("Estimated Y: " + apriltagTrans2d.getY());
      counter = 0;
    }

    //If our target distance is farther away from the Apriltag than we are currently
    if (destDistance > apriltagTrans2d.getX()) {
      directionInverse = -1;
    } else {
      //If the target distance is closer to the Apriltag than we are currently
      directionInverse = 1;
    }

    if (generalMethods.compareToTolerance(-yTolerance, yTolerance, apriltagTrans2d.getY(), true)) {

      if (apriltagTrans2d.getY() >= 0) {
        driveSpeedY = Constants.MAX_SPEED/6;
      } else {
        driveSpeedY = -Constants.MAX_SPEED/6;
      }

    } else {
      driveSpeedY = 0.0;
    }

    if (generalMethods.compareToTolerance((destDistance - xTolerance), (destDistance + xTolerance), apriltagTrans2d.getX(), true)) {

      if (apriltagTrans2d.getX() >= 0) {
        driveSpeedX = Constants.MAX_SPEED/6 * directionInverse;
      } else { 
        driveSpeedX = -Constants.MAX_SPEED/6 * directionInverse;
      }

    } else {
      driveSpeedX = 0.0;
    }

    if (Double.compare(driveSpeedX, 0.0) == 0 && 
        Double.compare(driveSpeedY, 0.0) == 0 && 
        !apriltagTrans2d.equals(new Translation2d())) { isFinishedFlag = true; }

    driveSpeeds = new ChassisSpeeds(driveSpeedX, driveSpeedY, 0);

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
