// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.movments;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.armConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
 

public class intakeWheels extends SubsystemBase {
  /** Creates a new intakeWheels. */
 private RelativeEncoder m_intakeEncoder;
  private CANSparkMax m_intakePickupWheels;
  private SparkPIDController m_PIDIntakePickUpWheels;
  private final DigitalInput m_noteSensor = new DigitalInput(0);


  public intakeWheels() {

    m_intakePickupWheels = new CANSparkMax(armConstants.intakePickUpWheels, MotorType.kBrushless);
    m_intakeEncoder = m_intakePickupWheels.getEncoder();
   
    m_PIDIntakePickUpWheels = m_intakePickupWheels.getPIDController();

    m_PIDIntakePickUpWheels.setP(.2);
    m_PIDIntakePickUpWheels.setI(0);
    m_PIDIntakePickUpWheels.setD(0);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public final boolean intake_wheels_speed_3000(){
      return (m_intakeEncoder.getVelocity()>2900);
  }

  public void intake_wheels_in(){
    m_intakePickupWheels.set(-.6);
   // m_PIDIntakePickUpWheels.setReference(3000,ControlType.kVelocity);
  };


  public void intake_wheels_out(){
    //m_PIDIntakePickUpWheels.setReference(-3000,ControlType.kVelocity);
    m_intakePickupWheels.set(.6);
  };


  public void intake_wheels_off(){
   // m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
    m_intakePickupWheels.set(0);
  };


public final boolean holdingNote(){
  return m_noteSensor.get();
  }
}


