// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flapExtendoGuyPneumatics;

public class FlapBreathe extends Command {
  /** Creates a new BreatheAir. */
  private final flapExtendoGuyPneumatics m_flapExtendoGuyPneumatics;
  private final Timer m_Timer;

  public FlapBreathe(flapExtendoGuyPneumatics ExtendoGuy, Timer wait) {
    m_flapExtendoGuyPneumatics = ExtendoGuy;
    m_Timer = wait;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flapExtendoGuyPneumatics.extendoGuy_set();
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
