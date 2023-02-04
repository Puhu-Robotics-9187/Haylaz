// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. 
   * @param x 
   * @param z 
   */
  public static CommandBase exampleAuto(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier z) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem,x,z));}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
