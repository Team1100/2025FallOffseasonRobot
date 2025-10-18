package frc.robot.commands.drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.SwerveDriveInputs;

public class SwerveDrive extends Command {
    public Drive m_Drive = null;
    private SwerveDriveInputs m_DriveInputs = null;
    private PIDController m_HeadingController;
    private final double kPHeading = 0.01;
    private final double kIHeading = 0;
    private final double kDHeading = 0;
    private final double kHeadingTolerance = 0.1;
    private boolean m_OperatorRotating = false;
    private TDNumber m_TDHeading;


    public SwerveDrive() {
        super(Drive.getInstance(), "Drive", "SwerveDrive");
        m_Drive = Drive.getInstance();
        m_DriveInputs = OI.getInstance().getSwerveDriveInputs();
        
        m_HeadingController = new PIDController(kPHeading, kIHeading, kDHeading);
        m_HeadingController.enableContinuousInput(-180,180);
        
        m_TDHeading = new TDNumber(m_Drive, "SwerveDrive", "TargetHeading");

        addRequirements(m_Drive);
    }

    @Override
    public void initialize() {
        m_OperatorRotating = false;
        m_TDHeading.set(m_Drive.getHeading());
    }

    @Override
    public void execute() {
        var rotationPower = -MathUtil.applyDeadband(m_DriveInputs.getRotation(), Constants.OIConstants.kDriveDeadband);
        if (rotationPower == 0) {
            if (m_OperatorRotating && 
                    MathUtil.isNear(0, m_Drive.getMeasuredSpeeds().omegaRadiansPerSecond, kHeadingTolerance)) {
                m_TDHeading.set(m_Drive.getHeading());
                m_OperatorRotating = false;
            }
            if (! m_OperatorRotating) {
                rotationPower = m_HeadingController.calculate(m_Drive.getHeading(),m_TDHeading.get());
            }
        }
        else {
            m_OperatorRotating = true;
        }
        var xPower = -MathUtil.applyDeadband(m_DriveInputs.getX(), Constants.OIConstants.kDriveDeadband);
        var yPower = -MathUtil.applyDeadband(m_DriveInputs.getY(), Constants.OIConstants.kDriveDeadband);

        m_Drive.drive(xPower, yPower, rotationPower, Constants.OIConstants.kDriveIsFieldRelative);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
