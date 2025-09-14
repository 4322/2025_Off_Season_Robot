package frc.robot.commands;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.elevator.Elevator;

public class CoastCommand extends Command {
  private Timer coastTimer = new Timer();
  private Deployer deployer;
  private Arm arm;
  private Elevator elevator;

  public CoastCommand(Arm arm, Elevator elevator, Deployer deployer) {
    this.deployer = deployer;
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(arm, elevator, deployer);
  }

  @Override
  public void initialize() {
    coastTimer.start();
    arm.setNeutralMode(IdleMode.kCoast);
    elevator.setNeutralMode(IdleMode.kCoast);
    deployer.setNeutralMode(IdleMode.kCoast);
  }

  @Override
  public void execute() {
   
  }

  @Override
  public boolean isFinished() {
    return coastTimer.hasElapsed(10) || DriverStation.isEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setNeutralMode(IdleMode.kBrake);
    elevator.setNeutralMode(IdleMode.kBrake);
    deployer.setNeutralMode(IdleMode.kBrake);
    coastTimer.stop();
    coastTimer.reset();
  }
}
