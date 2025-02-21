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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  private final SparkMax climberArm;

  public final RelativeEncoder climbEncoder;

  public final SparkClosedLoopController climbArmPidController;

  private final SparkMaxConfig climbArmConfig;

  public double kP, kI, kD, kv, kFF, kMaxOutput, kMinOutput;

  public climber() {
    climberArm = new SparkMax(13, MotorType.kBrushless);

    climbEncoder = climberArm.getEncoder();

    climbArmPidController = climberArm.getClosedLoopController();

    climbArmConfig = new SparkMaxConfig();

    kP = 0.01; 
    kI = 0;
    kD = .1; 
    kFF = .01;
    kMaxOutput = 1; 
    kMinOutput = -1;

    climbArmConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    climberArm.configure(climbArmConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void lowerArm(){
    climbArmPidController.setReference(8.4, ControlType.kPosition);
  }

  public void raiseArm(){
    climbArmPidController.setReference(0, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climb encoder", climbEncoder.getPosition());
  }
}
