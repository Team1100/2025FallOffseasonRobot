package frc.robot.commands.pivotything;
import frc.robot.subsystems.PivotyThing;
import frc.robot.testingdashboard.Command;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class AlgaeScoreAngle extends Command {
    private PivotyThing m_PivotyThing = null;

    public AlgaeScoreAngle() {
        super(PivotyThing.getInstance(), "PivotyThing", "AlgaeScoreAngle");
        m_PivotyThing = PivotyThing.getInstance();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_PivotyThing.setTargetAngle(Constants.PivotConstants.kAlgaeScoreAngle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_PivotyThing.getCurrentAngle(), m_PivotyThing.getTargetAngle(), Constants.PivotConstants.kPivotToleranceRadians);
    }
}