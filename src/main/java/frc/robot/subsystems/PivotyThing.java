// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class PivotyThing extends SubsystemBase {

  private static PivotyThing m_PivotyThing = null; 

  SparkMax m_PSparkMax;
  SparkMaxConfig m_SparkMaxConfig;
  RelativeEncoder m_pivotEncoder;
  SparkClosedLoopController m_pivotPidController;

  ArmFeedforward m_pivotFeedForward;
  TrapezoidProfile m_pivotProfile;
  TrapezoidProfile.State m_currentState;
  TrapezoidProfile.State m_targetState;

  double m_setAngle;

  TDNumber m_pivotCurrentOutput;

  private PivotyThing() {
    super("PivotyThing");

    if (RobotMap.P_ENABLED) {
      m_PSparkMax = new SparkMax(-1, MotorType.kBrushless);
      m_SparkMaxConfig = new SparkMaxConfig();
      m_SparkMaxConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pid(0, 0, 0);
      m_SparkMaxConfig.encoder.positionConversionFactor(Constants.PivotConstants.kPivotConversionFactor);
      m_PSparkMax.configure(m_SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
      m_pivotEncoder = m_PSparkMax.getEncoder();
      m_currentState = new TrapezoidProfile.State(getCurrentAngle(), 0);
      m_targetState = new TrapezoidProfile.State(getCurrentAngle(), 0);
      m_setAngle = getCurrentAngle();

      m_pivotCurrentOutput = new TDNumber(this, "Funnel", "Motor Current"); 

      m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0,0));
      m_pivotFeedForward = new ArmFeedforward(0, 0, 0);
    }
  }

  public static PivotyThing getInstance(){
    if (m_PivotyThing == null){
      m_PivotyThing = new PivotyThing();
    }
    return m_PivotyThing;
  }

  public void setTargetAngle(double angle) {
    double fixedAngle = MathUtil.clamp(angle, Constants.PivotConstants.kMinAngle, Constants.PivotConstants.kMaxAngle);
    if (fixedAngle != m_setAngle) {
      m_setAngle = fixedAngle;
      m_targetState = new TrapezoidProfile.State(fixedAngle, 0);
    }
  }
  
  public double getCurrentAngle() {
    return m_pivotEncoder.getPosition();
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(m_setAngle, getCurrentAngle(), Constants.PivotConstants.kPivotToleranceRadians);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotMap.P_ENABLED) {
      m_currentState = m_pivotProfile.calculate(Constants.robotPeriodTime, m_currentState, m_targetState);
      double calculatedFF = m_pivotFeedForward.calculate(m_currentState.position, m_currentState.velocity);
      m_pivotPidController.setReference(m_currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculatedFF);
    }
  }
}
