// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class PivotyThing extends SubsystemBase {

  private static PivotyThing m_PivotyThing = null; 
  
  SparkMax m_FLeftSparkMax;

  SparkMaxConfig m_SparkMaxConfig;

  private PivotyThing() {
    super("PivotyThing");

    if (-1.F_ENABLED) {
      m_FLeftSparkMax = new SparkMax(-1.F_LEFTMOTOR, MotorType.kBrushless);

      m_SparkMaxConfig = new SparkMaxConfig();
      
      //m_FLeftSparkMax.configure(m_SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); ?

      //m_FunnelCurrentOutput = new TDNumber(this, "Funnel", "Motor Current"); ?
    }
  }

  public static PivotyThing getInstance(){
    if (m_PivotyThing == null){
      m_PivotyThing = new PivotyThing();
    }
    return m_PivotyThing;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
