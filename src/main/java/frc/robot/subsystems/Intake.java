// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax deployMotor = new SparkMax(16,MotorType.kBrushless);
  private TalonFX intakeMotor = new TalonFX(26);

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDeploySpeed(double speed){
    deployMotor.set(speed);
  }
  
  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }
}
