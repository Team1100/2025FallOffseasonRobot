package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.ZeroHeading;
import frc.robot.commands.intake.Consume;
import frc.robot.commands.intake.Expel;
import frc.robot.commands.pivotything.AlgaeIntakeAngle;
import frc.robot.commands.pivotything.AlgaeScoreAngle;
import frc.robot.commands.pivotything.CoralIntakeAngle;
import frc.robot.commands.pivotything.CoralScoreAngle;
import frc.robot.commands.pivotything.Rezero;
import frc.robot.utils.SwerveDriveInputs;

public class OI {
    private static OI m_OI = null;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private SwerveDriveInputs m_DriveInputs;
    
    private OI() {
        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()){
            xInput=m_driverController::getLeftY;
            yInput=m_driverController::getLeftX;
        }
        else{
            xInput = ()->-m_driverController.getLeftX();
            yInput=m_driverController::getLeftY;
        }
        m_DriveInputs = new SwerveDriveInputs(xInput, yInput, m_driverController::getRightX);
    }

    public static OI getInstance(){
        if (m_OI == null){
            m_OI = new OI();
        }
        return m_OI;
    }
    

    public void configureBindings() {
        /***** Operator Bindings *****/
        m_operatorController.rightBumper().whileTrue(new Consume());
        m_operatorController.leftBumper().whileTrue(new Expel());
        m_operatorController.rightTrigger().whileTrue(new Expel());
        m_operatorController.leftTrigger().whileTrue(new Consume());
        //angles
        m_operatorController.a().onTrue(new CoralIntakeAngle());
        m_operatorController.y().onTrue(new CoralScoreAngle());
        m_operatorController.b().onTrue(new AlgaeIntakeAngle());
        m_operatorController.x().onTrue(new AlgaeScoreAngle());
        m_operatorController.start().onTrue(new Rezero());
        
        m_driverController.start().onTrue(new ZeroHeading());
    }

    public CommandXboxController getDriverController() {
        return m_driverController;
    }
    public CommandXboxController getOperatorController() {
        return m_operatorController;
    }
    public SwerveDriveInputs getSwerveDriveInputs() {
        return m_DriveInputs;
    }
}