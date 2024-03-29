// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.arm_position_commands.ArmTrapPosition;
import frc.robot.Commands.telescope_commands.TelescopeTrapPosition;
import frc.robot.subsystems.movments.armSubsystem;
import frc.robot.subsystems.movments.telescope;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapCommandGroup extends ParallelCommandGroup {
  /** Creates a new TrapCommandGroup. */
  public TrapCommandGroup(armSubsystem m_ArmSubsystem,telescope m_Telescope) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmTrapPosition(m_ArmSubsystem),
      new TelescopeTrapPosition(m_Telescope)

    );
  }
}
