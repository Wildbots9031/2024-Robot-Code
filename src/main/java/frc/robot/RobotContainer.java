// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.arm_position_commands.AmpCommandGroup;
import frc.robot.Commands.arm_position_commands.OverrideHoldCommandGroup;
import frc.robot.Commands.climb.ClimbDown;
import frc.robot.Commands.climb.ClimbUp;
//import frc.robot.Commands.command_groups.HoldCommandGroup;
import frc.robot.Commands.command_groups.IntakeCommandGroup;
import frc.robot.Commands.command_groups.PreClimbCommandGroup;
import frc.robot.Commands.command_groups.ShooterPositionGroup;
import frc.robot.Commands.command_groups.TrapCommandGroup;
//import frc.robot.Commands.intake_commands.IntakeWheelsInWithButton;
//import frc.robot.Commands.intake_commands.IntakeWheelsOffWithButton;
import frc.robot.Commands.intake_commands.IntakeWheelsOut;
import frc.robot.Commands.intake_commands.RepostionNote;
import frc.robot.Commands.shoot_commands.ShootNote;
import frc.robot.Commands.shoot_commands.TurnOffAllWheels;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.movments.DriveSubsystem;
import frc.robot.subsystems.movments.armSubsystem;
import frc.robot.subsystems.movments.climberSubsystem;
import frc.robot.subsystems.movments.intakeWheels;
import frc.robot.subsystems.movments.shooterWheels;
import frc.robot.subsystems.movments.telescope;
import frc.robot.subsystems.vision.LimeLightObject;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.limelight.autoAlign;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems. Subsystem Initialization
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final armSubsystem m_ArmSubsystem = new armSubsystem();
  private final telescope m_Telescope = new telescope();
  private final intakeWheels m_IntakeWheels = new intakeWheels();
  private final shooterWheels m_ShooterWheels = new shooterWheels();
  private final climberSubsystem m_ClimberSubsystem = new climberSubsystem();
  private final LimeLightObject m_limelightObject = new LimeLightObject();
  private final SendableChooser<Command> autoChooser;

 
  // The driver's controller
   CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
   CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  
 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

      //Register Commands

 //  NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
  NamedCommands.registerCommand("ShooterPositionGroup", new ShooterPositionGroup(m_ArmSubsystem,m_Telescope));
  NamedCommands.registerCommand("ShootNote", new ShootNote(m_IntakeWheels, m_ShooterWheels));
  NamedCommands.registerCommand("IntakeCommandGroup", new IntakeCommandGroup(m_ArmSubsystem, m_Telescope, m_IntakeWheels));
  NamedCommands.registerCommand("RepostionNote", new RepostionNote(m_IntakeWheels));
  NamedCommands.registerCommand("AmpCommandGroup", new AmpCommandGroup(m_ArmSubsystem,m_Telescope));
  NamedCommands.registerCommand("OverrideHoldCommandGroup", new OverrideHoldCommandGroup(m_ArmSubsystem,m_Telescope));



    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);


    //Autos
    SmartDashboard.putData("Auto 1", new PathPlannerAuto("Auto 1"));
    SmartDashboard.putData("Auto 2", new PathPlannerAuto("Auto 2"));



    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

  
            
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

  
      /*  new Trigger(m_ClimberSubsystem::leftHookSensor);//.onTrue(m_ClimberSubsystem.leftHookIsTouching());
    new Trigger(m_ClimberSubsystem::rightHookSensor);//.onTrue(m_ClimberSubsystem.rightHookIsTouching());

    if (new Trigger(m_ClimberSubsystem::rightHookSensor).equals(true)&&(new Trigger(m_ClimberSubsystem::leftHookSensor).equals(false))&&(m_driverController.a().equals(false))){
      m_ClimberSubsystem.rightHookIsTouching();
    }

    if (new Trigger(m_ClimberSubsystem::leftHookSensor).equals(true)&&(new Trigger(m_ClimberSubsystem::rightHookSensor).equals(false))&&(m_driverController.a().equals(false))){
      m_ClimberSubsystem.leftHookIsTouching();
    }

    if (new Trigger(m_ClimberSubsystem::rightHookSensor).equals(true)&&(new Trigger(m_ClimberSubsystem::leftHookSensor).equals(true))&&(m_driverController.a().equals(false))){
      m_ClimberSubsystem.climb();
    } 
    */

//set up Y button to auto align
    m_driverController.y().whileTrue(new autoAlign(m_limelightObject, m_robotDrive));
  
//Set up Y Button to Move to Shoot Position
    m_operatorController.y().onTrue(new ShooterPositionGroup(m_ArmSubsystem, m_Telescope));

//Set up B Button to Move to Hold Position
    m_driverController.b().onTrue(new OverrideHoldCommandGroup(m_ArmSubsystem, m_Telescope));

//Set up X Button to Move to Pre-Climb Position
    m_operatorController.x().onTrue(new PreClimbCommandGroup(m_ArmSubsystem, m_Telescope));

//Set up A Button to Move to Amp Position
    m_operatorController.a().onTrue(new AmpCommandGroup(m_ArmSubsystem, m_Telescope));

//Set up Start BUtton to Move to Intake Position
    m_operatorController.start().onTrue(new IntakeCommandGroup(m_ArmSubsystem, m_Telescope, m_IntakeWheels));

//Set up Left Trigger to Move to Trap Position
    m_operatorController.b().onTrue(new TrapCommandGroup(m_ArmSubsystem, m_Telescope));

//Set up Left Bumper to Get Off Chain
    m_driverController.leftBumper().onTrue(new ClimbUp(m_ClimberSubsystem));

//Set up Right Bumper to Climb
    m_driverController.rightBumper().onTrue(new ClimbDown(m_ClimberSubsystem));

 // Set up Right Trigger to Reverse Intake Wheels(Score in Amp)
   m_driverController.leftTrigger().whileTrue(new IntakeWheelsOut(m_IntakeWheels));

  //Setup left trigger on operator to shoot note
   m_driverController.rightTrigger().whileTrue(new ShootNote(m_IntakeWheels, m_ShooterWheels));

  //set up Turn Off All Wheels
   m_driverController.a().onTrue(new TurnOffAllWheels(m_ShooterWheels, m_IntakeWheels));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      0
    ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = m_robotDrive.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

         
          return autoChooser.getSelected();  
        
   /*  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);



    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    */
  }

}
