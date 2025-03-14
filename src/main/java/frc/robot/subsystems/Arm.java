// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX armkraken = new TalonFX(20);

  public Arm() {
    
    //arm motors
    armkraken.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void MoveArm(double speed){
    armkraken.set(speed);
  }
  public void StopArm(){
    MoveArm(0);
  }
}