package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake m_Intake = null;

    SparkMax m_ISparkMax = null;
    SparkMaxConfig m_SparkMaxConfig = null;

    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }

    private Intake(){
        super("Intake");
        if (RobotMap.I_ENABLED){
            m_ISparkMax = new SparkMax(-1, MotorType.kBrushless);
            m_SparkMaxConfig = new SparkMaxConfig();
            m_ISparkMax.configure(m_SparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }   
}

