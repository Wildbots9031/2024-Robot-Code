// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  /* 
  private CANSparkMax m_armRotationMotor;
  private CANSparkMax m_armTelescopingMotor;
  private CANSparkMax m_intakeRotationMotor;
  private CANSparkMax m_intakePickUpWheels;
  private CANSparkMax m_rightShooterMotor;
  private CANSparkMax m_leftShooterMotor;
  private SparkPIDController m_PIDArmRotation;
  private SparkPIDController m_PIDTelescope;
  private SparkPIDController m_PIDIntakeRotation;
  private SparkPIDController m_PIDRightShooter;
  private SparkPIDController m_PIDLeftShooter;
  private SparkPIDController m_PIDIntakePickUpWheels;
  */
  public armSubsystem() {

    /* 
   m_armRotationMotor = new CANSparkMax(1, MotorType.kBrushless);
   m_armTelescopingMotor = new CANSparkMax(2, MotorType.kBrushless);
   m_intakeRotationMotor = new CANSparkMax(3, MotorType.kBrushless);
   m_intakePickUpWheels = new CANSparkMax(4, MotorType.kBrushless);
   m_rightShooterMotor = new CANSparkMax(5, MotorType.kBrushless);
   m_leftShooterMotor = new CANSparkMax(6, MotorType.kBrushless);

   m_PIDArmRotation = m_armRotationMotor.getPIDController();
   m_PIDIntakeRotation = m_intakeRotationMotor.getPIDController();
   m_PIDTelescope = m_armTelescopingMotor.getPIDController();
   m_PIDRightShooter = m_rightShooterMotor.getPIDController();
   m_PIDLeftShooter = m_leftShooterMotor.getPIDController();
   m_PIDIntakePickUpWheels = m_intakePickUpWheels.getPIDController();

   m_PIDArmRotation.setP(1.0);
   m_PIDArmRotation.setI(0);
   m_PIDArmRotation.setD(0);

   m_PIDIntakeRotation.setP(1.0);
   m_PIDIntakeRotation.setI(0);
   m_PIDIntakeRotation.setD(0);

   m_PIDTelescope.setP(1.0);
   m_PIDTelescope.setI(0);
   m_PIDTelescope.setD(0);

   m_PIDRightShooter.setP(1.0);
   m_PIDRightShooter.setI(0);
   m_PIDRightShooter.setD(0);

   m_PIDLeftShooter.setP(1.0);
   m_PIDLeftShooter.setI(0);
   m_PIDLeftShooter.setD(0);

   m_PIDIntakePickUpWheels.setP(1.0);
   m_PIDIntakePickUpWheels.setI(0);
   m_PIDIntakePickUpWheels.setD(0);
   */

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*public Command intake_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command hold_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command amp_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command shoot_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command trap_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command pre_climb_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
m_PIDTelescope.setReference(1,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(1,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }*/
}
