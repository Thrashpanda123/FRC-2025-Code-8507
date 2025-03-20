// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */


  public final SparkMax intakeMotorTop;
  public final SparkMax intakeMotorBottom;

  private SparkClosedLoopController intakeTopPidController;
  private SparkClosedLoopController intakeBottomPidController;

  private SparkMaxConfig intakeMotorTopConfig;
  private SparkMaxConfig intakeMotorBottomConfig;

  public RelativeEncoder intakeMotorTopEncoder;
  public RelativeEncoder intakeMotorBottomEncoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private double setVolatgeIn; private double setVoltageOut;

  public Intake() {
    
    //setVolatgeIn = 5;
    //setVoltageOut = 5;

    intakeMotorTop = new SparkMax(9, MotorType.kBrushed);
    intakeMotorBottom = new SparkMax(10, MotorType.kBrushed);

    intakeMotorTopConfig = new SparkMaxConfig();
    intakeMotorBottomConfig = new SparkMaxConfig();
    /*
    intakeTopPidController = intakeMotorTop.getClosedLoopController();
    intakeBottomPidController = intakeMotorBottom.getClosedLoopController();

    intakeMotorTopEncoder = intakeMotorTop.getEncoder();
    intakeMotorBottomEncoder = intakeMotorBottom.getEncoder();


    kP = .002; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = .085; 
    kMaxOutput = .5; 
    kMinOutput = -1;
    maxRPM = 10000;
    
    intakeMotorTopConfig.closedLoop
      //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    intakeMotorBottomConfig.closedLoop
      //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);
    */
    intakeMotorBottomConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    intakeMotorTopConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    
    intakeMotorTop.configure(intakeMotorTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorBottom.configure(intakeMotorBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public void intakeIn(){
    intakeMotorTop.set(.55);
    intakeMotorBottom.set(.55);
  }

  public void intakeOut(){
    intakeMotorTop.set(-.55);
    intakeMotorBottom.set(-.55);
  }

  public void intakeStop(){
    intakeMotorTop.stopMotor();
    intakeMotorBottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
