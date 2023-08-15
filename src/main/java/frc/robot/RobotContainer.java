package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;

import frc.robot.commands.*;

public class RobotContainer {
    Joystick leftJoyStick = new Joystick(0);
    Joystick rightJoyStick = new Joystick(1);
    Trigger aimButton = new JoystickButton(leftJoyStick, 10);
    Trigger driveButton = new JoystickButton(leftJoyStick, 16);
    
    
    private Drive drive;

    public RobotContainer() {
        drive = new Drive();

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            drive.driveCommand(this::getLeftAxis, this::getRightAxis));
        aimButton.onTrue(new AimToTargetCommand(drive));
        driveButton.onTrue(new DriveToTargetCommand(10, drive));
    }

    private double getLeftAxis() {
        return -deadband(leftJoyStick.getY(), 0.05);
    }

    private double getRightAxis() {
        return -deadband(rightJoyStick.getX(), 0.05);
    }

    private static double deadband(double value, double tolerance) {
        if(Math.abs(value) < tolerance) {
            return 0.0;
        }

        return Math.copySign((value - tolerance)/(1.0-tolerance), value);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
