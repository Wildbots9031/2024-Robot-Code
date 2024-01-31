// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants.climbConstants;

public class climberSubsystem extends PIDSubsystem {
  /** Creates a new climberSubsystem. 
  */
  private CANSparkMax m_leftClimbMotor;
  private CANSparkMax m_rightClimbMotor;
  private SparkPIDController m_leftClimb;
  private SparkPIDController m_rightClimb;

  public climberSubsystem() {

    m_leftClimbMotor = new CANSparkMax(climbConstants.leftClimbMotorCanId, MotorType.kBrushless);
    m_rightClimbMotor = new CANSparkMax(climbConstants.rightClimbMotorCanId, MotorType.kBrushless);

    m_leftClimb = m_leftClimbMotor.getPIDController();
    m_rightClimb = m_rightClimbMotor.getPIDController();

   /*  super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0)); */
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }


}
