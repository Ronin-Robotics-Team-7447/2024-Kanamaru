// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical; 
  private boolean initRotation = false;

  /** Creates a new AbsoluteDrive. */
  public AbsoluteDrive(
    SwerveSubsystem swerve, 
    DoubleSupplier vX, 
    DoubleSupplier vY,
    DoubleSupplier headingHorizontal,
    DoubleSupplier headingVertical) 
    {
      this.swerve = swerve;
      this.vX = vX;
      this.vY = vY;
      this.headingHorizontal = headingHorizontal;
      this.headingVertical = headingVertical;
      addRequirements(swerve);    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Johnny assign boolean here
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
      vX.getAsDouble(), vY.getAsDouble(),
      headingHorizontal.getAsDouble(),
      headingVertical.getAsDouble());

    if (initRotation)
    {
      if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0)
      {
        Rotation2d firstLoopHeading = swerve.getHeading();

        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }

      initRotation = false;
    }

    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(
      translation,
      swerve.getFieldVelocity(), 
      swerve.getPose(),
      Constants.LOOP_TIME, 
      Constants.ROBOT_MASS, 
      List.of(Constants.CHASSIS),
      swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
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
