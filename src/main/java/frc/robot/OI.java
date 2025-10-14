package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

public class OI {
    private static OI m_OI = null;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    private OI() {}

    public static OI getInstance(){
        if (m_OI == null){
            m_OI = new OI();
        }
        return m_OI;
    }
    

    public void configureBindings() {
        
    }

    public CommandXboxController getDriverController() {
        return m_driverController;
    }
    public CommandXboxController getOperatorController() {
        return m_operatorController;
    }
}