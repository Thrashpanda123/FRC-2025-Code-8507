// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units.*;

import static edu.wpi.first.units.Units.Meter;

@SuppressWarnings("unused")
public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;

  private final boolean     visionDriveTest = true;

  private       Vision      vision;

  public SwerveSubsystem() {
      //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    if (visionDriveTest)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }

    setupPathPlanner();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }


  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void setupPathPlanner()
    
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward){
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } 
            else{
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()){
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } 
    catch (Exception e){
      // Handle exception as needed
      e.printStackTrace();
    }

  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance()){
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } 
    else{
      zeroGyro();
    }
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  
  public Command aimAtTarget(Cameras camera)
  {

    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent())
      {
        var result = resultO.get();
        if (result.hasTargets())
        {
          drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(result.getBestTarget()
                                                             .getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }



  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

}