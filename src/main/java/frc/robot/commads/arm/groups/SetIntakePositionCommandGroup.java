package frc.robot.commads.arm.groups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commads.arm.MoveArmToPositionCommand;
import frc.robot.commads.arm.MoveIntakeToPositionCommand;
import frc.robot.commads.arm.SetIntakePickUpSpeedCommand;
import frc.robot.commads.arm.SetShooterSpeedCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetIntakePositionCommandGroup extends ParallelCommandGroup {
    public SetIntakePositionCommandGroup(ArmSubsystem armSubsystem) {

        //This is a ParallelCommandGroup so everything happens at once
        super(new MoveArmToPositionCommand(armSubsystem, -13),
                new MoveIntakeToPositionCommand(armSubsystem, 0),
                new SetIntakePickUpSpeedCommand(armSubsystem, 100),
                new SetShooterSpeedCommand(armSubsystem, 0)
        );
    }
}