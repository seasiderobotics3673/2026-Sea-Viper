// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private SparkFlex shooterMotor = new SparkFlex(61, MotorType.kBrushless);

  private SparkFlexConfig shooterMotorConfig = new SparkFlexConfig();

  private double shooterMotorSpeed = 0.0;

  public Shooter() {
    shooterMotor.configure(shooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    shooterMotor.set(shooterMotorSpeed);
  }

  public void setSpeed(double shooterMotorSpeed) {
    this.shooterMotorSpeed = shooterMotorSpeed;
    shooterMotor.set(shooterMotorSpeed);
  }

}
