// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.movments;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants.armConstants;
import com.revrobotics.RelativeEncoder;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  
  private CANSparkMax m_armRotationMotor;
  private CANSparkMax m_intakeRotationMotor;
  private SparkPIDController m_PIDArmRotation;
  private SparkPIDController m_PIDIntakeRotation;
  private RelativeEncoder m_encoderArmRotationMotor;

  
  public armSubsystem() {

     
   m_armRotationMotor = new CANSparkMax(armConstants.armRotationMotor, MotorType.kBrushless);
   m_intakeRotationMotor = new CANSparkMax(armConstants.intakeRotationMotor, MotorType.kBrushless);

   m_PIDArmRotation = m_armRotationMotor.getPIDController();
   m_PIDIntakeRotation = m_intakeRotationMotor.getPIDController();
   m_encoderArmRotationMotor = m_armRotationMotor.getEncoder();

   m_PIDArmRotation.setP(.07);
   m_PIDArmRotation.setI(0);
   m_PIDArmRotation.setD(0);

   m_PIDIntakeRotation.setP(.6);
   m_PIDIntakeRotation.setI(0);
   m_PIDIntakeRotation.setD(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //set range for exiting Intake Position
  public final boolean arm_at_neg_14(){
    return (m_encoderArmRotationMotor.getPosition() > -13) && (m_encoderArmRotationMotor.getPosition() < -11);
  }

  //set range for exiting Amp Position
  public final boolean arm_at_pos_50(){
    return (m_encoderArmRotationMotor.getPosition() > 47) && (m_encoderArmRotationMotor.getPosition() < 53);
  }

  //set range for hold position
  public final boolean arm_at_65(){
    return (m_encoderArmRotationMotor.getPosition() > 48) && (m_encoderArmRotationMotor.getPosition() < 52);
  }

  //set range for Shooting Position
  public final boolean arm_at_pos18(){
    return (m_encoderArmRotationMotor.getPosition()> 17) && (m_encoderArmRotationMotor.getPosition()< 19);
  }

  //set range for Trap Scoring
  public final boolean arm_at_pos50(){
    return (m_encoderArmRotationMotor.getPosition()> 47) && (m_encoderArmRotationMotor.getPosition()< 52);
  }
   
  //Set range for zero
  public final boolean arm_at_pos0(){
    return (m_encoderArmRotationMotor.getPosition()> -2) && (m_encoderArmRotationMotor.getPosition()< 2);
  }


    public void intake_position(){

    m_PIDArmRotation.setReference(-12,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  
  public void hold_position(){

    m_PIDArmRotation.setReference(50,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };

 /*  public void override_hold_commands(){

    m_PIDArmRotation.setReference(0,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  */

  public void amp_position(){

    m_PIDArmRotation.setReference(65,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(-2.5,ControlType.kPosition);

    };
  
  public void shoot_position(){

    m_PIDArmRotation.setReference(18,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(-1.8,ControlType.kPosition);
  

    };

    public void shoot_podium_position(){

      m_PIDArmRotation.setReference(14,ControlType.kPosition);
      m_PIDIntakeRotation.setReference(-1.2,ControlType.kPosition);
    
  
      };
  
   public void trap_position(){

    m_PIDArmRotation.setReference(80,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(-6,ControlType.kPosition);

    };
  
  public void pre_climb_position(){

    m_PIDArmRotation.setReference(90,ControlType.kPosition);
    m_PIDIntakeRotation.setReference(0,ControlType.kPosition);

    };
  
  }
 

