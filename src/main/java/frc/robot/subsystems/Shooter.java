// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  StatusSignal<Temperature> launcherTemperature;
  double kickerTemperature;
  
  /** Creates a new Shooter. */
  //private TalonFX kickerMotor = new TalonFX(25);
  private SparkFlex kickerMotor = new SparkFlex(25, MotorType.kBrushless);
  private SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
  private TalonFX launcherMotor = new TalonFX(15);

  public Shooter() {
    kickerMotorConfig.idleMode(IdleMode.kBrake);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    launcherTemperature = launcherMotor.getDeviceTemp();
    kickerTemperature = kickerMotor.getMotorTemperature();
    SmartDashboard.putNumber("Launcher Temperature: ", Units.Fahrenheit.convertFrom(launcherTemperature.getValueAsDouble(), Celsius));
    SmartDashboard.putNumber("Kicker Temperature: ", Units.Fahrenheit.convertFrom(kickerTemperature, Celsius));
    SmartDashboard.updateValues();
  }

  public void setKickerMotorSpeed(double speed){
    kickerMotor.set(speed);
  } 

  public void setLauncherMotorSpeed(double speed){
    launcherMotor.set(speed);
  }
}
