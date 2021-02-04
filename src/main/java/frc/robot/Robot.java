
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//imports

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.*;




public class Robot extends TimedRobot {
  


  

    private static final int test_deviceCanID = 4;
    private CANSparkMax test_m_motor;
    private CANPIDController test_m_pidController;
    private CANEncoder test_m_encoder;
    public double test_kP, test_kI, test_kD, test_kIz, test_kFF, test_kMaxOutput, test_kMinOutput;


    @Override
    public void robotInit() {
      
        //examplePDP = new PowerDistributionPanel(0);
        // initialize motor
        test_m_motor = new CANSparkMax(test_deviceCanID, MotorType.kBrushless);

        /**
         * The restoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        test_m_motor.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        test_m_pidController = test_m_motor.getPIDController();

        // Encoder object created to display position values
        test_m_encoder = test_m_motor.getEncoder();
        test_m_encoder.setPosition(0);
        // PID coefficients
        test_kP = 0.1; 
        test_kI = 1e-4;
        test_kD = 1; 
        test_kIz = 0; 
        test_kFF = 0; 
        test_kMaxOutput = 1; 
        test_kMinOutput = -1;

       

        // set PID coefficients
        test_m_pidController.setP(test_kP);
        test_m_pidController.setI(test_kI);
        test_m_pidController.setD(test_kD);
        test_m_pidController.setIZone(test_kIz);
        test_m_pidController.setFF(test_kFF);
        test_m_pidController.setOutputRange(test_kMinOutput, test_kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", test_kP);
        SmartDashboard.putNumber("I Gain", test_kI);
        SmartDashboard.putNumber("D Gain", test_kD);
        SmartDashboard.putNumber("I Zone", test_kIz);
        SmartDashboard.putNumber("Feed Forward", test_kFF);
        SmartDashboard.putNumber("Max Output", test_kMaxOutput);
        SmartDashboard.putNumber("Min Output", test_kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
        //SmartDashboard.putNumber("Volts", examplePDP.getVoltage());
      
    }

    
    
  
    @Override
    public void teleopPeriodic() {
      
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != test_kP)) { test_m_pidController.setP(p); test_kP = p; }
        if((i != test_kI)) { test_m_pidController.setI(i); test_kI = i; }
        if((d != test_kD)) { test_m_pidController.setD(d); test_kD = d; }
        if((iz != test_kIz)) { test_m_pidController.setIZone(iz); test_kIz = iz; }
        if((ff != test_kFF)) { test_m_pidController.setFF(ff); test_kFF = ff; }
        if((max != test_kMaxOutput) || (min != test_kMinOutput)) { 
          test_m_pidController.setOutputRange(min, max); 
          test_kMinOutput = min; test_kMaxOutput = max; 
        }

        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.ControlType.kDutyCycle
         *  com.revrobotics.ControlType.kPosition
         *  com.revrobotics.ControlType.kVelocity
         *  com.revrobotics.ControlType.kVoltage
         */
        test_m_pidController.setReference(rotations, ControlType.kPosition);
        
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", test_m_encoder.getPosition());
        //SmartDashboard.putNumber("Volts", examplePDP.getVoltage());
    }

}