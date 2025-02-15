// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Driving1 Config
// Front: 1   3
// Back:  2   4

// This file is intended for PID control

package frc.robot.Subroutines;

//These libraries are neccessary for the motors to run. These are can bus motors, not pwm.
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import com.revrobotics.config.BaseConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.ClosedLoopConfig;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


public class Driving1 {
    SparkMax m_frontRight = new SparkMax(1, MotorType.kBrushless); //states the existance of a sparkmax with a CANid of 4
    SparkMax m_rearRight = new SparkMax(4, MotorType.kBrushless); //states the existance of a sparkmax with a CANid of 3
    SparkMax m_rearLeft = new SparkMax(2, MotorType.kBrushless); //states the existance of a sparkmax with a CANid of 1
    SparkMax m_frontLeft = new SparkMax(3, MotorType.kBrushless); //states the existance of a sparkmax with a CANid of 2

    
    SparkMax[] CANIDs= {m_frontRight, m_rearRight, m_rearLeft, m_frontLeft};
    
    DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

    RelativeEncoder d_Encoder_Left = m_frontLeft.getEncoder();

    public void Initial (){
        SparkConfigure.Drive_Initialize(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
        SparkClosedLoopController m_leaderPIDControl = m_frontLeft.getClosedLoopController();
        SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
        RelativeEncoder encoder1 = m_frontLeft.getEncoder(); // PID Controller Encoder

        m_leaderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        m_leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .p(0.1)
        .i(0.1)
        .d(0.1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        m_frontLeft.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void ManualDrive (double speed, double rotation, boolean square){
        double speedPercentage = SmartDashboard.getNumber("Drive Speed %", Robot.kDefaultSpeed); //range 0.0 to 1.0
        double rotationPercentage = 0.7; //range 0.0 to 1.0 
        m_drive.arcadeDrive(-speed*speedPercentage, -rotation*rotationPercentage, square);
    }

    public void AutoDrive (double speed, double rotation){
        double speedPercentage = 0.7; //range 0.0 to 1.0
        double rotationPercentage = 0.7; //range 0.0 to 1.0
        m_drive.arcadeDrive(-speed*speedPercentage, -rotation*rotationPercentage, false);
    }

    public double Feet2Steps (double Distance_Feet){ // poopy according to lorenzo
        double StepsPerRev = 8.475;
        double enc_WheelCurrent = getEncoder();
        double enc_WheelTarget = enc_WheelCurrent + (Distance_Feet*12*(1/(6*(Math.PI)))*StepsPerRev);
        return enc_WheelTarget;
    }

    public void TestMotor (double speed, int CAN_ID){
        //Used to test drive motors individually. Call on this function and supply speed and CAN_ID 1 - 4          
        if (CAN_ID <= 0 || CAN_ID >= CANIDs.length){
            String S = Integer.toString(CAN_ID);
            System.out.println("CAN_ID #" + S + " is not assigned to a drive motor");
        }
        else{
            CANIDs[(CAN_ID-1)].set(speed);
        }
    }

    public double getEncoder(){
        return d_Encoder_Left.getPosition();
    }
}
