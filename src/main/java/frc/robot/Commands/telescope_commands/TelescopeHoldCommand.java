// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.telescope_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.movments.telescope;

public class TelescopeHoldCommand extends Command {
  /** Creates a new HoldTelescopeCommand. */
  private final telescope m_telescope;
  
  public TelescopeHoldCommand(telescope m_telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_telescope = m_telescope;
    addRequirements(m_telescope);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_telescope.telescope_hold_postion();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_telescope.telescope_hold_postion();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescope.telescope_at_0();
  }
}
