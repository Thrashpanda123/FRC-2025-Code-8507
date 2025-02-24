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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax climberArmLeft, climberArmRight;

  public final RelativeEncoder climbEncoderLeft, climbEncoderRight;

  public final SparkClosedLoopController climbArmPidControllerRight, climbArmPidControllerLeft;

  private final SparkMaxConfig climbArmConfigRight, climbArmConfigLeft;

  public double kP, kI, kD, kv, kFF, kMaxOutput, kMinOutput;

  public Climber() {
    climberArmLeft = new SparkMax(13, MotorType.kBrushless);
    climberArmRight = new SparkMax(13, MotorType.kBrushless);

    climbEncoderLeft = climberArmLeft.getEncoder();
    climbEncoderRight = climberArmRight.getEncoder();

    climbArmPidControllerLeft = climberArmLeft.getClosedLoopController();
    climbArmPidControllerRight = climberArmRight.getClosedLoopController();

    climbArmConfigLeft = new SparkMaxConfig();
    climbArmConfigRight = new SparkMaxConfig();

    kP = 0.01; 
    kI = 0;
    kD = 1; 
    kFF = 1;
    kMaxOutput = 1; 
    kMinOutput = -1;

    climbArmConfigLeft.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    climbArmConfigRight.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

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
    climbArmPidControllerLeft.setReference(.5, ControlType.kPosition);
    climbArmPidControllerRight.setReference(-.5, ControlType.kPosition);
    //climbArmPidController.setReference(100, ControlType.kVelocity);
  }

  public void raiseArm(){
    climbArmPidControllerLeft.setReference(0, ControlType.kPosition);
    climbArmPidControllerRight.setReference(0, ControlType.kPosition);
    //climbArmPidController.setReference(-50, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climb encoder", climbEncoderLeft.getPosition());
    SmartDashboard.putNumber("climb encoder", climbEncoderRight.getPosition());
  }
}
