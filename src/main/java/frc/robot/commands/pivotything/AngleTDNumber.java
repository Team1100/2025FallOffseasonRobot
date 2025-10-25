package frc.robot.commands.pivotything;

import frc.robot.subsystems.PivotyThing;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;

public class AngleTDNumber extends Command {
    private PivotyThing m_PivotyThing = null;
    private TDNumber m_TdNumber = null;
    private double m_lastAngle = 0;
    public AngleTDNumber() {
        super(PivotyThing.getInstance(), "PivotyThing", "AngleTDNumber");
        m_PivotyThing = PivotyThing.getInstance();
        m_TdNumber = new TDNumber(PivotyThing.getInstance(), "Angle", "AngleTDNumber");
        m_lastAngle = m_TdNumber.get();
        addRequirements(m_PivotyThing);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double get = m_TdNumber.get();
        if (m_lastAngle != get) {
            m_lastAngle = get;
            m_PivotyThing.setTargetAngle(m_lastAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
