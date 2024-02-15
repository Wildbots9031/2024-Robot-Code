package frc.robot.commads;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;


public class AutoDriveExampleCommand extends Command {
    private final DriveSubsystem driveSubsystem;

    private SwerveControllerCommand swerveControllerCommand;
    private  Trajectory exampleTrajectory;

    private Pose2d startingPose;
    private List<Translation2d>  passThroughTranslation;
    private Pose2d finishPose;


    public AutoDriveExampleCommand(DriveSubsystem driveSubsystem, Pose2d start, List<Translation2d> passThrough, Pose2d finish) {
        this.driveSubsystem = driveSubsystem;
        this.startingPose = start;
        this.passThroughTranslation = passThrough;
        this.finishPose = finish;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                startingPose,
                passThroughTranslation,
                finishPose,
                config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                driveSubsystem::getPose, // Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                driveSubsystem::setModuleStates,
                driveSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }
}
