// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.movments.intakeWheels;

public class IntakeWheelsInWithButton extends Command {
  /** Creates a new IntakeWheelsWithButton. */

  private final intakeWheels m_intakeWheels;

  public IntakeWheelsInWithButton(intakeWheels intake_wheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeWheels = intake_wheels;
    addRequirements(m_intakeWheels);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
       m_intakeWheels.intake_wheels_in();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
           
    m_intakeWheels.intake_wheels_in();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  m_intakeWheels.intake_wheels_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return (m_intakeWheels.holdingNote());
  }
}
