// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final Translation3d FRONT_EDGE_TRANSLATION3D = new Translation3d(Units.inchesToMeters(14.25), Units.inchesToMeters(0), Units.inchesToMeters(2.25));
  public static final Translation3d BACK_EDGE_TRANSLATION3D = new Translation3d(Units.inchesToMeters(-14.25), Units.inchesToMeters(0), Units.inchesToMeters(2.25));
  public static final Translation3d CENTER_TRANSLATION3D = new Translation3d();

  public static final Translation2d FRONT_EDGE_TRANSLATION2D = new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(0));

  public static final double SWERVE_SPEED_FULL = 1.0;
  public static final double SWERVE_SPEED_SLOW = 0.5;


  
//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }



  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  //Measured as the distance from the floor to the center of the Apriltag, in meters.
  public static final double[] APRILTAG_HEIGHTS = 
  {
    FIELD_LAYOUT.getTagPose(1).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(2).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(3).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(4).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(5).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(6).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(7).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(8).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(9).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(10).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(11).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(12).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(13).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(14).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(15).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(16).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(17).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(18).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(19).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(20).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(21).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(22).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(23).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(24).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(25).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(26).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(27).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(28).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(29).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(30).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(31).get().getTranslation().getZ(),
    FIELD_LAYOUT.getTagPose(32).get().getTranslation().getZ(),
  };

  public static final Pose3d[] APRILTAG_POSES = {
    FIELD_LAYOUT.getTagPose(1).get(),
    FIELD_LAYOUT.getTagPose(2).get(),
    FIELD_LAYOUT.getTagPose(3).get(),
    FIELD_LAYOUT.getTagPose(4).get(),
    FIELD_LAYOUT.getTagPose(5).get(),
    FIELD_LAYOUT.getTagPose(6).get(),
    FIELD_LAYOUT.getTagPose(7).get(),
    FIELD_LAYOUT.getTagPose(8).get(),
    FIELD_LAYOUT.getTagPose(9).get(),
    FIELD_LAYOUT.getTagPose(10).get(),
    FIELD_LAYOUT.getTagPose(11).get(),
    FIELD_LAYOUT.getTagPose(12).get(),
    FIELD_LAYOUT.getTagPose(13).get(),
    FIELD_LAYOUT.getTagPose(14).get(),
    FIELD_LAYOUT.getTagPose(15).get(),
    FIELD_LAYOUT.getTagPose(16).get(),
    FIELD_LAYOUT.getTagPose(17).get(),
    FIELD_LAYOUT.getTagPose(18).get(),
    FIELD_LAYOUT.getTagPose(19).get(),
    FIELD_LAYOUT.getTagPose(20).get(),
    FIELD_LAYOUT.getTagPose(21).get(),
    FIELD_LAYOUT.getTagPose(22).get(),
    FIELD_LAYOUT.getTagPose(23).get(),
    FIELD_LAYOUT.getTagPose(24).get(),
    FIELD_LAYOUT.getTagPose(25).get(),
    FIELD_LAYOUT.getTagPose(26).get(),
    FIELD_LAYOUT.getTagPose(27).get(),
    FIELD_LAYOUT.getTagPose(28).get(),
    FIELD_LAYOUT.getTagPose(29).get(),
    FIELD_LAYOUT.getTagPose(30).get(),
    FIELD_LAYOUT.getTagPose(31).get(),
    FIELD_LAYOUT.getTagPose(32).get()
  };

  
}
