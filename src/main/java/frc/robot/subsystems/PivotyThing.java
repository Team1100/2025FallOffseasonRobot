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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.DioLimitSwitch;

public class PivotyThing extends SubsystemBase {

  private static PivotyThing m_PivotyThing = null; 

  SparkMax m_PSparkMax;
  SparkMaxConfig m_SparkMaxConfig;
  RelativeEncoder m_pivotEncoder;
  SparkClosedLoopController m_pivotPidController;

  DioLimitSwitch m_lowLimitSwitch;
  boolean m_closeLoopControlOn = true;

  ArmFeedforward m_pivotFeedForward;
  TrapezoidProfile m_pivotProfile;
  TrapezoidProfile.State m_currentState;
  TrapezoidProfile.State m_targetState;

  double m_setAngle;

  TDNumber td_pivotCurrentOutput;
  TDNumber td_pivotCurrentAngle;
  TDNumber td_pivotTargetAngle;
  TDNumber td_pivotVelocity;
  TDNumber td_pivotProfilePosition;
  TDNumber td_pivotProfileVelocity;
  
  double m_kP;
  TDNumber td_kP;
  double m_kI;
  TDNumber td_kI;
  double m_kD;
  TDNumber td_kD;
  double m_kS;
  TDNumber td_kS;
  double m_kG;
  TDNumber td_kG;
  double m_kV;
  TDNumber td_kV;

  private PivotyThing() {
    super("PivotyThing");

    if (RobotMap.P_ENABLED) {
      m_PSparkMax = new SparkMax(RobotMap.P_MOTOR_CAN_ID, MotorType.kBrushless);
      m_SparkMaxConfig = new SparkMaxConfig();
      m_SparkMaxConfig
        .inverted(false)//May need to change this
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 60);
      m_SparkMaxConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pid(Constants.PivotConstants.kP, Constants.PivotConstants.kI, Constants.PivotConstants.kD);
      m_SparkMaxConfig.encoder
        .positionConversionFactor(Constants.PivotConstants.kPivotConversionFactor)
        .velocityConversionFactor(Constants.PivotConstants.kPivotSpeedConversionFactor);

      m_pivotPidController = m_PSparkMax.getClosedLoopController();

      m_lowLimitSwitch = new DioLimitSwitch(RobotMap.P_LIMIT_SWITCH_DIO, true);

      m_PSparkMax.configure(m_SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_pivotEncoder = m_PSparkMax.getEncoder();
      m_currentState = new TrapezoidProfile.State(m_pivotEncoder.getPosition(), 0);
      m_targetState = new TrapezoidProfile.State(m_pivotEncoder.getPosition(), 0);
      m_setAngle = m_pivotEncoder.getPosition();

      td_pivotCurrentOutput = new TDNumber(this, "Motor", "Motor Current");
      td_pivotCurrentAngle = new TDNumber(this, "Position", "Current Angle");
      td_pivotTargetAngle = new TDNumber(this, "Position", "Target Angle");
      td_pivotVelocity = new TDNumber(this, "Position", "Current Velocity");
      td_pivotProfilePosition = new TDNumber(this, "Position", "Profile Angle");
      td_pivotProfileVelocity = new TDNumber(this, "Position", "Profile Velocity");

      td_kP = new TDNumber(this, "Tuning", "kP");
      td_kI = new TDNumber(this, "Tuning", "kI");
      td_kD = new TDNumber(this, "Tuning", "kD");
      td_kS = new TDNumber(this, "Tuning", "kS");
      td_kG = new TDNumber(this, "Tuning", "kG");
      td_kV = new TDNumber(this, "Tuning", "kV");

      m_pivotProfile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Constants.PivotConstants.kPivotMaxSpeedRadsPerSec, 
              Constants.PivotConstants.kPivotMaxAccelRadsPerSecSq));
      m_pivotFeedForward = new ArmFeedforward(Constants.PivotConstants.kS, Constants.PivotConstants.kG, Constants.PivotConstants.kV);
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

  public double getTargetAngle() {
    return m_setAngle;
  }
  
  public double getCurrentAngle() {
    return m_pivotEncoder.getPosition();
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(m_setAngle, getCurrentAngle(), Constants.PivotConstants.kPivotToleranceRadians);
  }

  private void handleLowLimitTriggered() {
    double angle = getCurrentAngle();
    if(m_PSparkMax.getAppliedOutput() < 0){
      m_PSparkMax.set(0);
    }
    if(!MathUtil.isNear(0, angle, Constants.PivotConstants.kPivotToleranceRadians)){
      m_pivotEncoder.setPosition(0);
    }
    if(m_setAngle < angle){
      m_currentState = new TrapezoidProfile.State(0, 0);
      setTargetAngle(0);
    }
  }

  public void reZero() {
    disableClosedLoop();
    setPivotSpeed(Constants.PivotConstants.kReZeroSpeed);
  }

  public void setPivotSpeed(double speed) {
    if(!m_closeLoopControlOn) {
      m_PSparkMax.set(speed);
    }
  }

  public void disableClosedLoop() {
    m_closeLoopControlOn = false;
  }
  public void enableClosedLoop() {
    double currentAngle = getCurrentAngle();
    m_currentState = new TrapezoidProfile.State(currentAngle, 0);
    setTargetAngle(currentAngle);
    m_closeLoopControlOn = true;
  }

  public boolean lowLimitHit() {
    return m_lowLimitSwitch.get();
  }

  private void updateTD() {
    td_pivotCurrentOutput.set(m_PSparkMax.getOutputCurrent());
    td_pivotCurrentAngle.set(getCurrentAngle());
    td_pivotTargetAngle.set(m_setAngle);
    td_pivotVelocity.set(m_pivotEncoder.getVelocity());
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    if(Constants.PivotConstants.kPivotTuningMode && m_PSparkMax != null) {
      boolean pidChanged = false;
      double tmp = td_kP.get();
      if(m_kP != tmp) {
        m_kP = tmp;
        m_SparkMaxConfig.closedLoop.p(m_kP);
        pidChanged = true;
      }
      tmp = td_kI.get();
      if(m_kI != tmp) {
        m_kI = tmp;
        m_SparkMaxConfig.closedLoop.i(m_kI);
        pidChanged = true;
      }
      tmp = td_kD.get();
      if(m_kD != tmp) {
        m_kD = tmp;
        m_SparkMaxConfig.closedLoop.d(m_kD);
        pidChanged = true;
      }
      if(pidChanged) {
        m_PSparkMax.configure(
            m_SparkMaxConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
      }

      boolean ffChanged = false;
      tmp = td_kS.get();
      if(m_kS != tmp) {
        m_kS = tmp;
        ffChanged = true;
      }
      tmp = td_kG.get();
      if(m_kG != tmp) {
        m_kG = tmp;
        ffChanged = true;
      }
      tmp = td_kV.get();
      if(m_kV != tmp) {
        m_kV = tmp;
        ffChanged = true;
      }
      if(ffChanged) {
        m_pivotFeedForward = new ArmFeedforward(m_kS, m_kG, m_kV);
      }
    }

    // This method will be called once per scheduler run
    if(RobotMap.P_ENABLED) {
      if(m_lowLimitSwitch.get()) {
        handleLowLimitTriggered();
      }
      if(m_closeLoopControlOn){
        m_currentState = m_pivotProfile.calculate(Constants.robotPeriodTime, m_currentState, m_targetState);
        td_pivotProfilePosition.set(m_currentState.position);
        td_pivotProfileVelocity.set(m_currentState.velocity);
        double calculatedFF = m_pivotFeedForward.calculate(m_currentState.position, m_currentState.velocity);
        m_pivotPidController.setReference(m_currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculatedFF);
      }
    }
    
    updateTD();
    super.periodic();
  }
}
