package frc.robot.commads.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class MoveIntakeToPositionCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double goToPosition;

    public MoveIntakeToPositionCommand(ArmSubsystem armSubsystem, double position) {
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
        armSubsystem.setIntakeRotation(goToPosition);
    }

    @Override
    public boolean isFinished() {
        //TODO you may want to add some tolerance in case the encoder becomes off over time.
        return armSubsystem.getIntakeRotation() == goToPosition;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
