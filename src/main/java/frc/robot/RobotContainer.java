// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.setArm;
import frc.robot.commands.setIntake;
import frc.robot.commands.stepArm;
import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  


  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  @SuppressWarnings("unused")
  private final XboxController mController = 
      new XboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
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

  //creating arm position commands
  private final Command armStart = new setArm(arm,0);
  private final Command armIntake = new setArm(arm,1);
  private final Command armL1 = new setArm(arm,2);
  private final Command armL2= new setArm(arm,3);
  private final Command armL3 = new setArm(arm,4);

  //creating intake commands
  private final Command intakeIn = new setIntake(intake, "in");
  private final Command intakeOut = new setIntake(intake, "out");
  private final Command intakeOff = new setIntake(intake, "off");
  
  
  private void configureBindings() {
    //arm position configs(dPad)
    m_driverController.povUp().onTrue(armIntake.andThen(intakeIn.until(() -> sensor.haveCoral())));
    m_driverController.povDown().onTrue(armL1);
    m_driverController.povRight().onTrue(armL2);
    m_driverController.povLeft().onTrue(armL3);
    m_driverController.start().onTrue(armStart);

    //intake button configs
    m_driverController.x().onTrue(intakeIn.until(() -> sensor.haveCoral()).andThen(intakeOff));
    m_driverController.y().onTrue(intakeOut.until(() -> !sensor.haveCoral()).andThen(intakeOff));

    
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  }

