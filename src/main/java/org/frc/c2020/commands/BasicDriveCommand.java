package org.frc.c2020.commands;

import org.frc.c2020.subsystems.DrivetrainSubsystem;
import org.frc.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Vector2 translation;
  private final double rotation;
  private final boolean fieldOriented;

  public BasicDriveCommand(
      DrivetrainSubsystem drivetrain, Vector2 translation, double rotation, boolean fieldOriented) {
    this.drivetrain = drivetrain;
    this.translation = translation;
    this.rotation = rotation;
    this.fieldOriented = fieldOriented;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(translation, rotation, fieldOriented);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0.0, false);
  }
}