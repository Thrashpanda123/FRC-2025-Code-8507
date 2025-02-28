// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.TimerTask;
import java.util.Timer;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeOutAuto extends Command {
  /** Creates a new intakeOut. */
  Intake intake;
  private boolean finished = false;

  public intakeOutAuto(Intake i) {
    intake = i;

    addRequirements(i);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;

    TimerTask task = new TimerTask() {
      public void run() {
        finished = true;
      }
    };

    Timer timer = new Timer();

    timer.schedule(task, 1000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
