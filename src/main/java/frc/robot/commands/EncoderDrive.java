// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
// Using encoders this command makes the robot travel a certain distance
public class EncoderDrive extends CommandBase {
  /** Creates a new EncoderDrive. */
  DriveTrain dt;
  Double setpoint; 
  public EncoderDrive(DriveTrain dt, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.setpoint = setpoint;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(setpoint > dt.TicksToMeters()){
    dt.tankDrive(0.2, 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dt.TicksToMeters() >= setpoint;

  }
}
