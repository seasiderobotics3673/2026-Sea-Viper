// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  //private TalonFX kickerMotor = new TalonFX(25);
  private SparkFlex kickerMotor = new SparkFlex(25, MotorType.kBrushless);
  private SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
  private TalonFX launcherMotor = new TalonFX(15);

  public Shooter() {
    //kickerMotor.configure(kickerMotorConfig, null, null)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setKickerMotorSpeed(double speed){
    kickerMotor.set(speed);
  } 

  public void setLauncherMotorSpeed(double speed){
    launcherMotor.set(speed);
  }
}
