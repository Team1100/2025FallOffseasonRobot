package frc.robot.commands.pivotything;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotyThing;
import frc.robot.testingdashboard.Command;

public class PivotManualControl extends Command{
    private PivotyThing m_PivotyThing = null;
    CommandXboxController m_XboxController = null;

    public PivotManualControl() {
        super(PivotyThing.getInstance(), "PivotyThing", "PivotManualControl");
        m_PivotyThing = PivotyThing.getInstance();
        m_XboxController = OI.getInstance().getOperatorController();
        addRequirements(m_PivotyThing);
    }

    @Override
    public void initialize() {
        m_PivotyThing.disableClosedLoop();
    }

    @Override
    public void execute() {
        double power = MathUtil.applyDeadband(
            m_XboxController.getLeftY() * PivotConstants.kPivotManualSpeedFactor,
            Constants.OIConstants.kDriveDeadband);
        m_PivotyThing.setPivotSpeed(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_PivotyThing.setPivotSpeed(0);
        m_PivotyThing.enableClosedLoop();
    }
}
