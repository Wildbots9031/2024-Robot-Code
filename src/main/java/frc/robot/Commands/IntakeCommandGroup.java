// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.telescope;
import frc.robot.subsystems.intakeWheels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class IntakeCommandGroup extends SequentialCommandGroup {
  /** Creates a new SequentialIntakeCommandGroup. */
  public IntakeCommandGroup(armSubsystem m_ArmSubsystem,telescope m_Telescope, intakeWheels m_IntakeWheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmIntakeCommand(m_ArmSubsystem),
     new TelescopeIntakeCommand(m_Telescope),
     new IntakeWheelsIn(m_IntakeWheels)
     );
  }
}
