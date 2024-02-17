// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.armConstants;
import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  
  private CANSparkMax m_armRotationMotor;
  private CANSparkMax m_intakeRotationMotor;
  private SparkPIDController m_PIDArmRotation;
  private SparkPIDController m_PIDIntakeRotation;
  private final DigitalInput m_climberSensor = new DigitalInput(1);
  private RelativeEncoder m_encoderArmRotationMotor;

  
  public armSubsystem() {

     
   m_armRotationMotor = new CANSparkMax(armConstants.armRotationMotor, MotorType.kBrushless);
   m_intakeRotationMotor = new CANSparkMax(armConstants.intakeRotationMotor, MotorType.kBrushless);

   m_PIDArmRotation = m_armRotationMotor.getPIDController();
   m_PIDIntakeRotation = m_intakeRotationMotor.getPIDController();
   m_encoderArmRotationMotor = m_armRotationMotor.getEncoder();

   m_PIDArmRotation.setP(.1);
   m_PIDArmRotation.setI(0);
   m_PIDArmRotation.setD(0);

   m_PIDIntakeRotation.setP(.1);
   m_PIDIntakeRotation.setI(0);
   m_PIDIntakeRotation.setD(0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
//public final boolean getAsBoolean()

  public final boolean climbingButton(){
    return m_climberSensor.get();
  }

  public final boolean arm_at_neg_14(){
    return (m_encoderArmRotationMotor.getPosition() > -15) && (m_encoderArmRotationMotor.getPosition() < -12);
  }

  public final boolean arm_at_pos_60(){
    return (m_encoderArmRotationMotor.getPosition() > 55) && (m_encoderArmRotationMotor.getPosition() < 65);
  }

  public void intake_position(){

m_PIDArmRotation.setReference(-13,ControlType.kPosition);
m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  

  
 
    public final boolean retract_note(){
      return ((m_encoderArmRotationMotor.getPosition() > -15) && (m_encoderArmRotationMotor.getPosition() < -12));
      }

    
 /*    m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
    m_PIDArmRotation.setReference(1,ControlType.kPosition); 
    m_PIDTelescope.setReference(0,ControlType.kPosition);
    m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
    m_PIDRightShooter.setReference(0,ControlType.kVelocity);
    */

     
      
      

    

  
  

   public void hold_position(){

  m_PIDArmRotation.setReference(0,ControlType.kPosition);
  m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  


  public void amp_position(){

m_PIDArmRotation.setReference(60,ControlType.kPosition);
m_PIDIntakeRotation.setReference(-3,ControlType.kPosition);

    };
  

   public void score_note_amp(){
    };
  

  public void shoot_position(){

m_PIDArmRotation.setReference(60,ControlType.kPosition);
m_PIDIntakeRotation.setReference(-1,ControlType.kPosition);

    };
  
 
   public void shoot_note(){
    
    };
  

  public void trap_position(){

m_PIDArmRotation.setReference(1,ControlType.kPosition);
m_PIDIntakeRotation.setReference(1,ControlType.kPosition);

    };
  

   public void score_note_trap(){
  }

  public void pre_climb_position(){

m_PIDArmRotation.setReference(80,ControlType.kPosition);
m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  
  }
 

