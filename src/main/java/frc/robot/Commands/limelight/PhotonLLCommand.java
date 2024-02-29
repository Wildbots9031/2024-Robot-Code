package frc.robot.Commands.limelight;

import frc.robot.subsystems.vision.PhotonLL;

//import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;

public class PhotonLLCommand extends Command {

  private final PhotonLL photonCamera;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PhotonLLCommand() {
    photonCamera = PhotonLL.getInstance();
    
    addRequirements(photonCamera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    photonCamera.periodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}