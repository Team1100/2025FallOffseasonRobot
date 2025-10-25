// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotything;

import frc.robot.testingdashboard.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.PivotyThing;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotManualAngleControl extends Command {
  PivotyThing m_pivot;
  CommandXboxController m_operatorController;

  /** Creates a new PivotManualAngleControl. */
  public PivotManualAngleControl() {
    super(PivotyThing.getInstance(), "Commands", "ManualAngleControl");
    m_pivot = PivotyThing.getInstance();
    m_operatorController = OI.getInstance().getOperatorController();

    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotAngle = m_pivot.getTargetAngle();
    double input = MathUtil.applyDeadband(m_operatorController.getLeftY(), Constants.OIConstants.kOperatorDeadband);

    pivotAngle += input * Constants.PivotConstants.kPivotMaxAngleIncrementRads;
    m_pivot.setTargetAngle(pivotAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
