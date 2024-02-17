// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;

public class ArmHoldPosition extends Command {
  private final armSubsystem m_armSubsystem;

  /** Creates a new HoldPosition. */
  public ArmHoldPosition(armSubsystem arm_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_armSubsystem = arm_subsystem;

    addRequirements(m_armSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_armSubsystem.hold_position();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
