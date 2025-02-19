// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {
  public final DigitalInput intakeSensor;
  private final DigitalInput armLowLimit;
  public Sensors() {
    intakeSensor = new DigitalInput(0);
    armLowLimit = new DigitalInput(1);
  }


  public boolean haveCoral(){
    if(intakeSensor.get()){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
