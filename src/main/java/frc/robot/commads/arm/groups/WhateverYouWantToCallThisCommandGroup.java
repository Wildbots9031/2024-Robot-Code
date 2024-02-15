package frc.robot.commads.arm.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commads.arm.MoveIntakeToPositionCommand;
import frc.robot.commads.arm.MoveTelescopeToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;

public class WhateverYouWantToCallThisCommandGroup extends SequentialCommandGroup {
    public WhateverYouWantToCallThisCommandGroup(ArmSubsystem armSubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new MoveTelescopeToPositionCommand(armSubsystem, 0),
                new SetIntakePositionCommandGroup(armSubsystem),
                new MoveIntakeToPositionCommand(armSubsystem, 0)
        );

    }
}