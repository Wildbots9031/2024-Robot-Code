// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants.armConstants;
import com.revrobotics.RelativeEncoder;


public class shooterWheels extends SubsystemBase {
  /** Creates a new shooterWheels. */

  private CANSparkMax m_rightShooterMotor;
  private CANSparkMax m_leftShooterMotor;

  private SparkPIDController m_PIDRightShooter;
  private SparkPIDController m_PIDLeftShooter;

  private RelativeEncoder m_encoderRightShootMotor;


  public shooterWheels() {

    m_rightShooterMotor = new CANSparkMax(armConstants.rightShooterMotor, MotorType.kBrushless);
    m_leftShooterMotor = new CANSparkMax(armConstants.leftShooterMotor, MotorType.kBrushless);
 
    m_PIDRightShooter = m_rightShooterMotor.getPIDController();
    m_PIDLeftShooter = m_leftShooterMotor.getPIDController();

    m_PIDRightShooter.setP(.2);
    m_PIDRightShooter.setI(0);
    m_PIDRightShooter.setD(0);
 
    m_PIDLeftShooter.setP(.2);
    m_PIDLeftShooter.setI(0);
    m_PIDLeftShooter.setD(0);   
 
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
// set range for exiting ShooterWheelsOn
public final boolean shooterVelocity(){
  return m_encoderRightShootMotor.getVelocity()>2800;
}

public void shootOn() {
 
  m_PIDLeftShooter.setReference(-3000,ControlType.kVelocity);
  m_PIDRightShooter.setReference(3000,ControlType.kVelocity);
  
}

public void shootOff() {
 
  m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
  m_PIDRightShooter.setReference(0,ControlType.kVelocity);
  
}

}
