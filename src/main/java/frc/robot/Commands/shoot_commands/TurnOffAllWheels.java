// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.shoot_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.movments.intakeWheels;
import frc.robot.subsystems.movments.shooterWheels;

public class TurnOffAllWheels extends Command {
  /** Creates a new TurnOffAllWheels. */

  private final shooterWheels m_ShooterWheels;
  private final intakeWheels m_IntakeWheels;

  public TurnOffAllWheels(shooterWheels shooter_Wheels, intakeWheels intake_Wheels) {
    // Use addRequirements() here to declare subsystem dependencies.
this.m_ShooterWheels = shooter_Wheels;
this.m_IntakeWheels = intake_Wheels;

addRequirements(m_ShooterWheels, m_IntakeWheels);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_IntakeWheels.intake_wheels_off();
    m_ShooterWheels.shootOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_IntakeWheels.intake_wheels_off();
    m_ShooterWheels.shootOff();

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
