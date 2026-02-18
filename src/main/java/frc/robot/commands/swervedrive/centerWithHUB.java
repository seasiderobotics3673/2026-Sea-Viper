// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.auto.AutoWait;
import frc.robot.commands.swervedrive.drivebase.rotateToHeading;
import frc.robot.commands.swervedrive.vision.moveToTargetDistance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class centerWithHUB extends SequentialCommandGroup {
  /** Creates a new centerWithHUB. */
  public centerWithHUB(SwerveSubsystem drivebase, double destinationDistance, Vision vision, int fiducialId, Translation3d offsetPointBot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new rotateToHeading(drivebase, Rotation2d.fromDegrees(0)),
      new AutoWait(100),
      new InstantCommand(()-> drivebase.zeroGyro()),
      new moveToTargetDistance(3, drivebase, vision, fiducialId, offsetPointBot, new Translation3d(), 0),
      new rotateToHeading(drivebase, Rotation2d.fromDegrees(0))
    );
  }
}
