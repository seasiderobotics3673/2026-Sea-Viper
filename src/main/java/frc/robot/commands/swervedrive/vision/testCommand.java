// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testCommand extends InstantCommand {

  private Vision vision;
  private Cameras cameraEnum;
  private Translation3d offsetPoint;
  private boolean isSpecificID;
  private int fiducialId;
  private SwerveSubsystem drivebase;

  public testCommand(Cameras cameraEnum, Translation3d offsetPoint, boolean isSpecificID, int fiducialId, Vision vision, SwerveSubsystem drivebase) {
    this.cameraEnum = cameraEnum;
    this.offsetPoint = offsetPoint;
    this.isSpecificID = isSpecificID;
    this.fiducialId = fiducialId;
    this.vision = vision;
    this.drivebase = drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("getAllTargetTransforms: " + vision.getAllTargetTransforms(cameraEnum));
    //System.out.println("getTargetPos: " + vision.getTargetTransform(cameraEnum, isSpecificID, fiducialId).getTranslation() + " " + drivebase.getHeading());
    //System.out.println("");
    //System.out.println("getTargetPosOffset: " + vision.getTargetTransformOffset(cameraEnum, offsetPoint, isSpecificID, fiducialId).getTranslation());
  }
}
