// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.swervedrive.drivebase.simpleDrive;
import frc.robot.commands.swervedrive.vision.moveNearTargetPoint;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.subsystems.swervedrive.altDriveCommand;
import swervelib.SwerveInputStream;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MainAuto extends SequentialCommandGroup {
  /** Creates a new MainAuto. */
  public MainAuto(Shooter shooter, Intake intake, SwerveSubsystem drivebase, Vision vision, Cameras camera, SwerveInputStream controllerInput, RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new simpleDrive(drivebase, new Translation2d(-1, 0)).withTimeout(1.75),
      new AutoWait(200),
      new InstantCommand(()-> intake.setDeploySpeed(1)),
      new AutoWait(100),
      new InstantCommand(()-> intake.setDeploySpeed(0)),
      new altDriveCommand(drivebase, vision, camera, controllerInput, robotContainer).withTimeout(1),
      new moveNearTargetPoint(drivebase, vision, camera, 2),
      new AutoWait(300),
      new InstantCommand(()-> shooter.setLauncherMotorSpeed(-0.65)),
      new AutoWait(3000),
      new InstantCommand(()-> shooter.setKickerMotorSpeed(0.7)),
      new AutoWait(350),
      new InstantCommand(()-> shooter.setKickerMotorSpeed(-1)),
      new AutoWait(5500),
      new InstantCommand(()-> shooter.setKickerMotorSpeed(0)),
      new InstantCommand(()-> shooter.setLauncherMotorSpeed(0))
      
    );
  }
}
