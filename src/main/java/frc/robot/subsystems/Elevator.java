// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax leftWinch = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax rightWinch = new SparkMax(22, MotorType.kBrushless);
  /** Creates a new Elevator. */
  public Elevator() {
    
    //winch motors
    //rightWinch.follow(leftWinch);
    rightWinch.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ExtendElevator(double speed){
    leftWinch.set(speed);
  }
  public void StopElevator(){
    ExtendElevator(0);
  }
}