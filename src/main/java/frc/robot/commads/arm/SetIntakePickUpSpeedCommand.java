package frc.robot.commads.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class SetIntakePickUpSpeedCommand extends Command {
    private final ArmSubsystem armSubsystem;

    private final double intakeSpeed;

    public SetIntakePickUpSpeedCommand(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.intakeSpeed = speed;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armSubsystem.setIntakePickUpWheelsSpeed(intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        //TODO you may want to add some tolerance
        return armSubsystem.getIntakePickUpWheelsSpeed() == intakeSpeed;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
