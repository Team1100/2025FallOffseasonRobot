// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.SwerveDrive;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.testingdashboard.TestingDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotMap.init();
    
    // initialize the drive (and the AutoBuilder)
    Drive.getInstance().setDefaultCommand(new SwerveDrive());

    // the AutoBuilder is configured in the Drive constructor. That must be done first.
    m_autoChooser = AutoBuilder.buildAutoChooser("Center Auto");
    new TDSendable(Drive.getInstance(), "Auto Commands", "Chooser", m_autoChooser);

    // Configure the trigger bindings
    OI.getInstance().configureBindings();

    // Create testing dashboard
    TestingDashboard.getInstance().createTestingDashboard();
    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return m_autoChooser.getSelected();
  }
}
