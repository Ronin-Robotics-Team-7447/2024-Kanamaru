// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.littleBillyRELOAD;

public class OperationReloadMag extends Command {
  /** Creates a new OperationReloadMag. */
  private final littleBillyRELOAD m_littleBillyRELOAD;

  public OperationReloadMag(littleBillyRELOAD reload) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_littleBillyRELOAD = reload;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_littleBillyRELOAD.inhale(Constants.IntakeConstants.Reloader_speed);
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
