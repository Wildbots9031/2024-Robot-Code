package frc.robot.commads.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class MoveTelescopeToPositionCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double goToPosition;

    public MoveTelescopeToPositionCommand(ArmSubsystem armSubsystem, double position) {
        this.armSubsystem = armSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.goToPosition = position;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armSubsystem.setTelescopePosition(goToPosition);
    }

    @Override
    public boolean isFinished() {
        //TODO you may want to add some tolerance in case the encoder becomes off over time.
        //Or another thing we do is add a limit switch at the bottom and when we hit that switch we know we are at 0
        return armSubsystem.getTelescopePosition() == goToPosition;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
