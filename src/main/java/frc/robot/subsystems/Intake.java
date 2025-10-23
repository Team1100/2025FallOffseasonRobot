package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Intake extends SubsystemBase{
    private static Intake m_Intake = null;

    SparkMax m_ISparkMax = null;
    SparkMaxConfig m_SparkMaxConfig = null;

    TDNumber td_currentOutput;

    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }

    private Intake(){
        super("Intake");
        if (RobotMap.I_ENABLED){
            m_ISparkMax = new SparkMax(RobotMap.I_MOTOR_CAN_ID, MotorType.kBrushless);
            m_SparkMaxConfig = new SparkMaxConfig();
            m_SparkMaxConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(25, 60);
            m_ISparkMax.configure(m_SparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            td_currentOutput = new TDNumber(this, "Current", "Current");
        }
    }   

    public void spinIn(double speed) {
        if (m_ISparkMax != null) {
            m_ISparkMax.set(speed);
        }
    }
    public void spinOut(double speed) {
        if (m_ISparkMax != null) {
            m_ISparkMax.set(-1 * speed);
        }
    }
    public void stop() {
        if (m_ISparkMax != null) {
            // If you change this to positive zero I will drive the robot into your leg
            m_ISparkMax.set(-0);
        }
    }

    @Override
    public void periodic() {
        if (RobotMap.I_ENABLED) {
            td_currentOutput.set(m_ISparkMax.getOutputCurrent());
        }
        super.periodic();
    }
}

