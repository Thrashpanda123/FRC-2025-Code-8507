// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.setArm;
import frc.robot.commands.Autos;
import frc.robot.commands.homingSequence;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.commands.intakeIn;
import frc.robot.commands.intakeOut;
import frc.robot.subsystems.Sensors;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem arm = new ArmSubsystem();
  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final Intake intake = new Intake();
  private final Sensors sensor = new Sensors();
  private final climber climber = new climber();
  private final Wrist wrist = new Wrist();

  //intake commands
  private final Command Intake = new intakeIn(intake);
  private final Command Outtake = new intakeOut(intake);
  private final Command OuttakeAuto = new SequentialCommandGroup( new InstantCommand(intake::intakeOut), new WaitCommand(.2), new InstantCommand(intake::intakeStop));

  //creating arm position commands
  private final Command armStart = new setArm(arm,0);
  private final Command armIntake = new ParallelCommandGroup(new setArm(arm,1), new InstantCommand(wrist::setWristOpen));
  private final Command armL1 = new ParallelCommandGroup(new setArm(arm,2),new InstantCommand(wrist::setWristClose));
  private final Command armL2= new setArm(arm,3);
  private final Command armL3 = new setArm(arm,4);
  private final Command score = new setArm(arm, 5);
  private final Command homingSequence = new homingSequence(arm);


  //climb commands
  private final Command climbUp = new InstantCommand(climber::raiseArm);
  private final Command climbDown = new InstantCommand(climber::lowerArm);


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  @SuppressWarnings("unused")
  private final XboxController mController = new XboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {
    configureBindings();

    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("name", Commands.print("I exist"));
    NamedCommands.registerCommand("Intake",  Intake);
    //NamedCommands.registerCommand("Outtake", Commands.sequence(Commands.runOnce(() -> intake.intakeOut()), Commands.waitSeconds(1), Commands.runOnce(() -> intake.intakeStop())));
    NamedCommands.registerCommand("Outtake", OuttakeAuto);
    NamedCommands.registerCommand("armL1", armL1);
    NamedCommands.registerCommand("armStart", armStart);
    NamedCommands.registerCommand("armL2", armL2);
    NamedCommands.registerCommand("armIntake", armIntake);
    NamedCommands.registerCommand("WristOpen", wrist.setWristOpen());
    NamedCommands.registerCommand("WristClosed", wrist.setWristClose());
    new EventTrigger("Intake Coral").onTrue(Intake);

  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.5)
                                                                .allianceRelativeControl(true);
  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  Command driveFieldOrientatedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);


  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  
  public void configureBindings() {
    //arm position configs(dPad)
    m_driverController.povDown().onTrue(armIntake);
    m_driverController.povRight().onTrue(score);
    m_driverController.povUp().onTrue(armL1);
    m_driverController.povLeft().onTrue(armL2);
    m_driverController.start().onTrue(homingSequence.until(() -> sensor.isHomed()));
    //m_driverController.start().onTrue(armStart);

    //intake button configs
    m_driverController.x().whileTrue(Intake.until(() -> sensor.haveCoral()));
    m_driverController.y().whileTrue(Outtake);
    
    //Climber binds
    m_driverController.rightBumper().onTrue(climbUp);
    m_driverController.leftBumper().onTrue(climbDown);

    //Wrist binds
    m_driverController.b().onTrue(wrist.setWristOpen());
    m_driverController.a().onTrue(wrist.setWristClose());
    
  }

  public void robotContainerPerodic() {
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return driveBase.getAutonomousCommand("Blue Middle");
  }
}

