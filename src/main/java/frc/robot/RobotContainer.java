// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.ShootNote;
import frc.robot.Commands.ShooterPositionGroup;
import frc.robot.Commands.ShooterWheelsOff;
import frc.robot.Commands.AmpCommandGroup;
import frc.robot.Commands.PreClimbCommandGroup;
import frc.robot.Commands.RepostionNote;
import frc.robot.Commands.HoldCommandGroup;
import frc.robot.Commands.IntakeCommandGroup;
import frc.robot.Commands.TrapCommandGroup;
import frc.robot.Commands.TurnOffAllWheels;
import frc.robot.Commands.IntakeWheelsOut;
import frc.robot.Commands.ClimbUp;
import frc.robot.Commands.ClimbDown;
import frc.robot.Commands.OverrideHoldCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeWheels;
import frc.robot.subsystems.shooterWheels;
import frc.robot.subsystems.telescope;
import frc.robot.subsystems.climberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final armSubsystem m_ArmSubsystem = new armSubsystem();
  private final telescope m_Telescope = new telescope();
  private final intakeWheels m_IntakeWheels = new intakeWheels();
  private final shooterWheels m_ShooterWheels = new shooterWheels();
  private final climberSubsystem m_ClimberSubsystem = new climberSubsystem();
  private final SendableChooser<Command> autoChooser;
 
  

  // The driver's controller
   CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
   CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("ShooterPositionGroup", new ShooterPositionGroup(m_ArmSubsystem,m_Telescope));
    NamedCommands.registerCommand("ShootNote", new ShootNote(m_IntakeWheels, m_ShooterWheels));
    NamedCommands.registerCommand("IntakeCommandGroup", new IntakeCommandGroup(m_ArmSubsystem, m_Telescope, m_IntakeWheels));
    NamedCommands.registerCommand("RepostionNote", new RepostionNote(m_IntakeWheels));
    NamedCommands.registerCommand("AmpCommandGroup", new AmpCommandGroup(m_ArmSubsystem,m_Telescope));
    NamedCommands.registerCommand("OverrideHoldCommandGroup", new OverrideHoldCommandGroup(m_ArmSubsystem,m_Telescope));
NamedCommands.registerCommand("ShooterWheelsOff", new ShooterWheelsOff(m_ShooterWheels));

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Auto 1", new PathPlannerAuto("Auto 1"));
    SmartDashboard.putData("Auto 2", new PathPlannerAuto("Auto 2"));
    SmartDashboard.putData("score From Center", new PathPlannerAuto("score from center"));
    SmartDashboard.putData("score far note", new PathPlannerAuto("score far note"));
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



  //Set up Limit to Swith to Move to Hold
    if (new Trigger(m_ArmSubsystem::arm_at_neg_14).getAsBoolean() && (new Trigger(m_IntakeWheels::holdingNote).getAsBoolean())) 
      {new HoldCommandGroup(m_ArmSubsystem, m_Telescope, m_IntakeWheels);
    }
  //Set up Right Trigger to Shoot Note into Speaker
   if (new Trigger(m_driverController.rightTrigger()).getAsBoolean() && (new Trigger(m_ArmSubsystem::arm_at_pos40).getAsBoolean())) 
      {new ShootNote(m_IntakeWheels, m_ShooterWheels);}

  //Set up Right Trigger to Score Amp
   if (new Trigger(m_driverController.rightTrigger()).getAsBoolean() && (new Trigger(m_ArmSubsystem::arm_at_pos_60).getAsBoolean())) 
      {new IntakeWheelsOut(m_IntakeWheels);}

  //Set up Right Trigger to Score Trap
    if (new Trigger(m_driverController.rightTrigger()).getAsBoolean() && (new Trigger(m_ArmSubsystem::arm_at_pos50).getAsBoolean())) 
      {new IntakeWheelsOut(m_IntakeWheels);}

/*<<<<<<< HEAD
    //Set up Y Button to Move to Shoot Position
    m_operatorController.y().onTrue(new ShooterPositionGroup(m_ArmSubsystem, m_Telescope));
=======
    m_driverController.y().onTrue(m_ArmSubsystem.shoot_position());
    m_driverController.b().onTrue(m_ArmSubsystem.hold_position());
    m_driverController.x().onTrue(m_ArmSubsystem.pre_climb_position());
    m_driverController.a().onTrue(m_ArmSubsystem.amp_position());
<<<<<<< HEAD
    
    m_driverController.start().onTrue(
      parallel(
        
            (m_ArmSubsystem.telescope_hold_postion()), 

   waitUntil(m_ArmSubsystem.armEncoderPosition())
    .andThen
   
            (m_ArmSubsystem.intake_position())));
=======
    m_driverController.start().onTrue(m_ArmSubsystem.telescope_hold_postion().andThen(m_ArmSubsystem.intake_position()).andThen(m_ArmSubsystem.telescope_intake_position()));
    m_driverController.leftTrigger().onTrue(m_ArmSubsystem.shoot_note());
//>>>>>>> a22d2001d93c830e2b7e16362d4efd228eaa7721
//>>>>>>> b5bd453742da80ebbaee5b5b6f325afdeceb285f*/

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


   

           

            SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

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
    // Create config for trajectory
    /*TrajectoryConfig config = new TrajectoryConfig(
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));*/
  }

}
