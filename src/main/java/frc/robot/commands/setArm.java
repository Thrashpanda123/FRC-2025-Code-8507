// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@SuppressWarnings("unused")
public class setArm extends Command {
  /** Creates a new ArmControl. */
  ArmSubsystem arm;
  int Level = 0;
  boolean isFinished = false;

  public setArm(ArmSubsystem a, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    Level = level;

    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Level == 0)
      arm.setLevel(0);
    else if(Level == 1){
      arm.setLevel(1);
      if(arm.armRight_encoder.getPosition() == Constants.intakePos){
        isFinished = true;
      }
    }
    else if(Level == 2){
      arm.setLevel(2);
      if(arm.armRight_encoder.getPosition() == Constants.L1_scorePos){
          isFinished = true;
      }
    }
    else if(Level == 3){
      arm.setLevel(3);
      if(arm.armRight_encoder.getPosition() == Constants.L2_scorePos){
        isFinished = true;
      }
    }
    else if(Level == 5){
      arm.setLevel(5);
      if(arm.armRight_encoder.getPosition() == Constants.score){
        isFinished = true;
      }
    }
    else
      arm.setLevel(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
