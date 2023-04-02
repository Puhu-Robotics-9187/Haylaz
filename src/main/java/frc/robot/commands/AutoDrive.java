// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrive extends CommandBase {
  DriveSubsystem m_drive;
  double startTime;
  boolean finished = false;
  double speed = 0.8;
  /** Creates a new AutoDrive. */
  public AutoDrive(DriveSubsystem drive) {
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Timer.getFPGATimestamp() - startTime) < 1){
      m_drive.haylazDrive.arcadeDrive(speed, 0);
    } else if((Timer.getFPGATimestamp() - startTime) > 1 && (Timer.getFPGATimestamp() - startTime) < 2){
      m_drive.haylazDrive.arcadeDrive(-speed, 0);
    } else if((Timer.getFPGATimestamp() - startTime) > 2 && (Timer.getFPGATimestamp() - startTime) < 3.5){
      m_drive.haylazDrive.arcadeDrive(speed, 0);
    } else {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
