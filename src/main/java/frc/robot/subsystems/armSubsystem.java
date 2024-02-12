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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.armConstants;
import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  
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
  private final DigitalInput m_noteSensor = new DigitalInput(0);
  private final DigitalInput m_climberSensor = new DigitalInput(1);
  private RelativeEncoder m_encoderArmRotationMotor;
  
  public armSubsystem() {

     
   m_armRotationMotor = new CANSparkMax(armConstants.armRotationMotor, MotorType.kBrushless);
   m_armTelescopingMotor = new CANSparkMax(armConstants.armTelescopingMotor, MotorType.kBrushless);
   m_intakeRotationMotor = new CANSparkMax(armConstants.intakeRotationMotor, MotorType.kBrushless);
   m_intakePickUpWheels = new CANSparkMax(armConstants.intakePickUpWheels, MotorType.kBrushless);
   m_rightShooterMotor = new CANSparkMax(armConstants.rightShooterMotor, MotorType.kBrushless);
   m_leftShooterMotor = new CANSparkMax(armConstants.leftShooterMotor, MotorType.kBrushless);

   m_PIDArmRotation = m_armRotationMotor.getPIDController();
   m_PIDIntakeRotation = m_intakeRotationMotor.getPIDController();
   m_PIDTelescope = m_armTelescopingMotor.getPIDController();
   m_PIDRightShooter = m_rightShooterMotor.getPIDController();
   m_PIDLeftShooter = m_leftShooterMotor.getPIDController();
   m_PIDIntakePickUpWheels = m_intakePickUpWheels.getPIDController();
   m_encoderArmRotationMotor = m_armRotationMotor.getEncoder();

   m_PIDArmRotation.setP(.1);
   m_PIDArmRotation.setI(0);
   m_PIDArmRotation.setD(0);

   m_PIDIntakeRotation.setP(.1);
   m_PIDIntakeRotation.setI(0);
   m_PIDIntakeRotation.setD(0);

   m_PIDTelescope.setP(.3);
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
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
//public final boolean getAsBoolean()
  public final boolean holdingNote(){
    return m_noteSensor.get();
  }

  public final boolean climbingButton(){
    return m_climberSensor.get();
  }

  public Command intake_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(-13,ControlType.kPosition);
m_PIDIntakeRotation.setReference(0,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(100,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

  public Command telescope_intake_position(){

    return runOnce(() ->{
  m_PIDTelescope.setReference(-9,ControlType.kPosition);
    
    });
  }
  
  public Command telescope_hold_postion(){

    return runOnce(() ->{
  m_PIDTelescope.setReference(0,ControlType.kPosition);   

    });
  }
 
    public Command retract_note(){
      return runOnce(() ->{ if 
        ((holdingNote()==true) && (m_encoderArmRotationMotor.getPosition() > -15) && (m_encoderArmRotationMotor.getPosition() < -12)) {
        m_PIDIntakePickUpWheels.setReference(-10,ControlType.kVelocity);
      } else {
        m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
      }

    
 /*    m_PIDIntakeRotation.setReference(1,ControlType.kPosition);
    m_PIDArmRotation.setReference(1,ControlType.kPosition); 
    m_PIDTelescope.setReference(0,ControlType.kPosition);
    m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
    m_PIDRightShooter.setReference(0,ControlType.kVelocity);
    */
});
     
      
      

    }

  
  

   public Command hold_position(){

    return runOnce(() ->{
  m_PIDArmRotation.setReference(0,ControlType.kPosition);
  m_PIDIntakeRotation.setReference(0,ControlType.kPosition);
  m_PIDTelescope.setReference(0,ControlType.kPosition);
  m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
  m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
  m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }


  public Command amp_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(60,ControlType.kPosition);
m_PIDIntakeRotation.setReference(-3,ControlType.kPosition);
m_PIDTelescope.setReference(-4,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

   public Command score_note_amp(){
    return runOnce(() ->{
m_PIDIntakePickUpWheels.setReference(-100,ControlType.kVelocity);    
    });
  }

  public Command shoot_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(60,ControlType.kPosition);
m_PIDIntakeRotation.setReference(-1,ControlType.kPosition);
m_PIDTelescope.setReference(0,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

   public Command shoot_note(){
    return runOnce(() ->{
      m_PIDLeftShooter.setReference(1,ControlType.kVelocity);
      m_PIDRightShooter.setReference(1,ControlType.kVelocity);
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

   public Command score_note_trap(){
    return runOnce(() ->{});
  }

  public Command pre_climb_position(){

    return runOnce(() ->{
m_PIDArmRotation.setReference(80,ControlType.kPosition);
m_PIDIntakeRotation.setReference(0,ControlType.kPosition);
m_PIDTelescope.setReference(0,ControlType.kPosition);
m_PIDIntakePickUpWheels.setReference(0,ControlType.kVelocity);
m_PIDLeftShooter.setReference(0,ControlType.kVelocity);
m_PIDRightShooter.setReference(0,ControlType.kVelocity);

    });
  }

 
}
