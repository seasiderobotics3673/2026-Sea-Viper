// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  private double destAngle;
  private double tolerance;
  private ChassisSpeeds driveSpeed;
  private int counter;


  /** Creates a new moveToTargetDistance. */
  public moveToTargetDistance(double destDistance, SwerveSubsystem drivebase, Vision vision, int fiducialId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destDistance = destDistance;
    this.drivebase = drivebase;
    this.fiducialId = fiducialId;
    this.vision = vision;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (fiducialId == -1 || fiducialId == 0) {
      isRegardingSpecificID = false;
    } else {
      isRegardingSpecificID = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter++;

    if (counter >= 20) {
      System.out.println(apriltagTrans2d.getY());
      counter = 0;
    }

    apriltagTrans2d = vision.getTargetPos(cameraEnum, isRegardingSpecificID, fiducialId);

    //if (-0.1 >= apriltagTrans2d.getY() || apriltagTrans2d.getY() >= 0.1) {
    if (apriltagTrans2d.getY() <= -0.08 || apriltagTrans2d.getY() >= 0.08) {
      if (apriltagTrans2d.getY() >= 0) {
        driveSpeed = new ChassisSpeeds(0, Constants.MAX_SPEED/6, 0);
      } else {
        driveSpeed = new ChassisSpeeds(0, -Constants.MAX_SPEED/6, 0);
      }
    } else {
      driveSpeed = new ChassisSpeeds(0,0,0);
    }
    drivebase.drive(driveSpeed);
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
