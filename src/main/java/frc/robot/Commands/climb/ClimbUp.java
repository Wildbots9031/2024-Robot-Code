// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.movments.climberSubsystem;

public class ClimbUp extends Command {
  private final climberSubsystem m_ClimberSubsystem;

  /** Creates a new ClimbUp. */
  public ClimbUp(climberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ClimberSubsystem = climberSubsystem;

    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ClimberSubsystem.climbUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ClimberSubsystem.climbUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
