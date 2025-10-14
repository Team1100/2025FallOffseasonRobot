package frc.robot.commands.drive;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.SwerveDriveInputs;

public class SwerveDrive extends Command {
    public Drive m_Drive = null;
    private SwerveDriveInputs m_DriveInputs = null;

    public SwerveDrive() {
        super(Drive.getInstance(), "Drive", "SwerveDrive");
        m_Drive = Drive.getInstance();
        m_DriveInputs = OI.getInstance().getSwerveDriveInputs();

        addRequirements(m_Drive);
    }
}
