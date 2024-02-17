// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climbConstants;

public class climberSubsystem extends SubsystemBase {
    /**
     * Creates a new climberSubsystem.
     */
    private CANSparkMax m_leftClimbMotor;
    private CANSparkMax m_rightClimbMotor;
    private SparkPIDController m_PIDleftClimb;
    private SparkPIDController m_PIDrightClimb;
    private final DigitalInput m_leftHookSensor = new DigitalInput(2);
    private final DigitalInput m_rightHookSensor = new DigitalInput(3);
    private final DigitalInput m_leftBassSensor = new DigitalInput(4);
    private final DigitalInput m_rightBassSensor = new DigitalInput(5);

    public climberSubsystem() {

        m_leftClimbMotor = new CANSparkMax(climbConstants.leftClimbMotorCanId, MotorType.kBrushless);
        m_rightClimbMotor = new CANSparkMax(climbConstants.rightClimbMotorCanId, MotorType.kBrushless);

        m_PIDleftClimb = m_leftClimbMotor.getPIDController();
        m_PIDleftClimb.setP(1.0);
        m_PIDleftClimb.setI(0);
        m_PIDleftClimb.setD(0);

        m_PIDrightClimb = m_rightClimbMotor.getPIDController();
        m_PIDrightClimb.setP(1.0);
        m_PIDrightClimb.setI(0);
        m_PIDrightClimb.setD(0);

    }

    public final boolean leftBassSensor() {
        return m_leftBassSensor.get();
    }

    public final boolean rightBassSensor() {
        return m_rightBassSensor.get();
    }

    public final boolean leftHookSensor() {
        return m_leftHookSensor.get();
    }

    public final boolean rightHookSensor() {
        return m_rightHookSensor.get();
    }


    public void climb() {
//TODO needs coded
    }

    public boolean leftHookIsTouching() {
        //TODO needs coded
        return  false;
    }

    public boolean rightHookIsTouching() {
        //TODO needs coded
      return  false;
    }

    public boolean rightBassIsTouching() {
        //TODO needs coded
        return  false;
    }

    public boolean leftBassIsTouching() {
        //TODO needs coded
        return  false;
    }

    public void dissmountChain() {
        //TODO needs coded
    }


}
