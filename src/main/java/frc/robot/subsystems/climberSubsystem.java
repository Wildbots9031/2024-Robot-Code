// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants.climbConstants;

public class climberSubsystem extends SubsystemBase {
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

  }

}
