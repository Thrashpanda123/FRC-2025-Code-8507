// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {
  public final DigitalInput intakeSensor;
  public final DigitalInput HomingLimit;
  public Sensors() {
    intakeSensor = new DigitalInput(0);
    HomingLimit = new DigitalInput(1);
  }


  public boolean haveCoral(){
    if(intakeSensor.get()){
      return false;
    }
    else{
      return true;
    }
  }

  public boolean isHomed(){
    if(!HomingLimit.get()){
      return false;
    }
    else{
      return true;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Homing limit switch", isHomed());
    SmartDashboard.putBoolean("Have Coral?", haveCoral());
  }
}
