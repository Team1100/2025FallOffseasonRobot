// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.SwerveDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotyThing;
import frc.robot.testingdashboard.TestingDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive m_drive;
  private final PivotyThing m_pivot;
  private final Intake m_intake;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotMap.init();
    
    m_drive = Drive.getInstance();
    m_drive.setDefaultCommand(new SwerveDrive());
    m_pivot = PivotyThing.getInstance();
    m_intake = Intake.getInstance();


    // Configure the trigger bindings
    OI.getInstance().configureBindings();

    // Create testing dashboard
    TestingDashboard.getInstance().createTestingDashboard();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {return new InstantCommand();}
}
