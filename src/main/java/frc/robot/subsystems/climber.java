// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  private final SparkMax climberArmLeft, climberArmRight;

  public final RelativeEncoder climbEncoderLeft;

  public final SparkClosedLoopController climbArmPidControllerLeft;

  private final SparkMaxConfig climbArmConfigRight, climbArmConfigLeft;

  public double kP, kI, kD, kv, kFF, kMaxOutput, kMinOutput;

  public ArmSubsystem arm;

  public climber() {
    climberArmLeft = new SparkMax(13, MotorType.kBrushless);
    climberArmRight = new SparkMax(14, MotorType.kBrushless);

    climbEncoderLeft = climberArmLeft.getEncoder();
    //climbEncoderRight = climberArmRight.getEncoder();

    climbArmPidControllerLeft = climberArmLeft.getClosedLoopController();
    //climbArmPidControllerRight = climberArmRight.getClosedLoopController();

    climbArmConfigLeft = new SparkMaxConfig();
    climbArmConfigRight = new SparkMaxConfig();

    kP = .1; 
    kI = 0;
    kD = 0; 
    kFF = 1;
    kMaxOutput = 1; 
    kMinOutput = -1;
    
    climbArmConfigLeft
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    climbArmConfigLeft.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD)
      //.velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    climbArmConfigRight
      .idleMode(IdleMode.kBrake)
      .follow(climberArmLeft, false);
    /*
    climbArmConfigRight.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kP,kI,kD)
      //.velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);
    */

    climberArmLeft.configure(climbArmConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    climberArmRight.configure(climbArmConfigRight, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public Command armRaise(){
    return runOnce(() -> raiseArm());
  }

  public Command armLower(){
    return runOnce(() -> lowerArm());
  }

  public void lowerArm(){
    climbArmPidControllerLeft.setReference(0, ControlType.kPosition);
    //climbArmPidControllerRight.setReference(0, ControlType.kPosition);
    //climbArmPidController.setReference(100, ControlType.kVelocity);
  }

  public void raiseArm(){
    climbArmPidControllerLeft.setReference(100, ControlType.kPosition);
  }

  public boolean armUp(){
    if(climbEncoderLeft.getPosition() == 42)
      return true;
    else  
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climb State", armUp());
    SmartDashboard.putNumber("climb encoder", climbEncoderLeft.getPosition());
  }
}
