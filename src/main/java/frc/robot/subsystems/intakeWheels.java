// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;


public class intakeWheels extends SubsystemBase {
    /**
     * Creates a new intakeWheels.
     */

    private CANSparkMax m_intakePickupWheels;
    private SparkPIDController m_PIDIntakePickUpWheels;
    private final DigitalInput m_noteSensor = new DigitalInput(0);


    public intakeWheels() {

        m_intakePickupWheels = new CANSparkMax(armConstants.intakePickUpWheels, MotorType.kBrushless);

        m_PIDIntakePickUpWheels = m_intakePickupWheels.getPIDController();

        m_PIDIntakePickUpWheels.setP(1.0);
        m_PIDIntakePickUpWheels.setI(0);
        m_PIDIntakePickUpWheels.setD(0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intake_wheels_in() {
        m_PIDIntakePickUpWheels.setReference(1000, ControlType.kVelocity);
    }


    public void intake_wheels_out() {
        m_PIDIntakePickUpWheels.setReference(-1000, ControlType.kVelocity);
    }

    public void intake_wheels_off() {

        m_PIDIntakePickUpWheels.setReference(0, ControlType.kVelocity);

    }

    public final boolean holdingNote() {
        return m_noteSensor.get();
    }

}
