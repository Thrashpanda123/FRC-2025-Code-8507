// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armLeft;
  private final SparkMax armRight;
  
  public RelativeEncoder armLeft_encoder;
  public RelativeEncoder armRight_encoder;
  
  private SparkClosedLoopController armLeft_pidController;
  private SparkClosedLoopController armRight_pidController;

  private SparkMaxConfig rightMotorConfig;
  private SparkMaxConfig leftMotorConfig;

  public double kP, kI, kD, kv, kFF, kMaxOutput, kMinOutput;


  public ArmSubsystem() {
    armLeft = new SparkMax(11, MotorType.kBrushless);
    armRight = new SparkMax(12, MotorType.kBrushless);

    armLeft_encoder = armLeft.getEncoder();
    armRight_encoder = armRight.getEncoder();

    armRight_pidController = armRight.getClosedLoopController();
    armLeft_pidController = armLeft.getClosedLoopController();

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    kP = 0.001; 
    kI = 0;
    kD = 0; 
    kFF = .1;
    kMaxOutput = 1; 
    kMinOutput = -1;
    
    leftMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    rightMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);
    
    armLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    armRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getRightPosition(){
    return armRight_encoder.getPosition();
  }

  public double getLeftPosition(){
    return armLeft_encoder.getPosition();
  }

  public void step(double stepAmount){
    armRight_pidController.setReference(-stepAmount, ControlType.kPosition);
    armLeft_pidController.setReference(stepAmount, ControlType.kPosition);
  }
  public void setLevel(int level) {
    if(level == 0) {
      armRight_pidController.setReference(Constants.startPos, ControlType.kPosition);
      armLeft_pidController.setReference(Constants.startPos, ControlType.kPosition);
    } else if(level == 1){
      armRight_pidController.setReference(Constants.intakePos, ControlType.kPosition);
      armLeft_pidController.setReference(-Constants.intakePos, ControlType.kPosition);
    }else if(level == 2) {
      armRight_pidController.setReference(Constants.L1_scorePos, ControlType.kPosition);
      armLeft_pidController.setReference(-Constants.L1_scorePos, ControlType.kPosition);
    } else if(level == 3) {
      armRight_pidController.setReference(Constants.L2_scorePos, ControlType.kPosition);
      armLeft_pidController.setReference(-Constants.L2_scorePos, ControlType.kPosition);
    } else if(level == 4) {
      armRight_pidController.setReference(Constants.L3_scorePos, ControlType.kPosition);
      armLeft_pidController.setReference(-Constants.L3_scorePos, ControlType.kPosition);
    }
  }

  public boolean isIntakePos(){
    if(armLeft_encoder.getPosition() == 1200 && armRight_encoder.getPosition() == -1200)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftEncoder", getLeftPosition());
    SmartDashboard.putNumber("rightEncoder", getRightPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
