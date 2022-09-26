// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class Straight extends CommandBase {

  private Limelight limelight;
  private Drive drive;

  // PID output = KP*error + KI*errorSum + KD*rate;
  private double kP = 0.2;
  private double kI = 0.1;
  private double kD = 0.1;
  private double iLimit = 0.18;
  private double error;
  private double errorSum;
  private double rate;
  private double output;
  private double dT;

  private double lastTimeStamp;

  /** Creates a new Move. */
  public Straight(Limelight limelight, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drive = drive;
    addRequirements(limelight, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setMotorToZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getV() == 1) {
      dT = Timer.getFPGATimestamp() - lastTimeStamp;
      error = limelight.getY();

      if (Math.abs(error) < iLimit) {
        errorSum += error * dT;
      }

      output = kP * error + kI * errorSum + kD * rate;

      drive.arcadeDrive(output, 0);
    } else {
      drive.arcadeDrive(0.5, 0);
    }

    lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotorToZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
