package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AimToTargetCommand;
import frc.robot.commands.DriveAndAimTargetCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriveToTargetCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
    
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);

    Trigger myDriveForwardCommand = new JoystickButton(rightJoystick, 2);
    Trigger myTurnToAngleCommand = new JoystickButton(leftJoystick, 2);
    private Drive drive; 
    private LimelightInterface limelightInterface; 
    private RamseteAutoBuilder autoBuilder;

    public RobotContainer(){
        drive = new Drive();

        autoBuilder = new RamseteAutoBuilder(
            drive::getPose,
            drive::resetOdometry, 
            new RamseteController(),
            drive.kinematics,
            new SimpleMotorFeedforward(0, 2.6),
            drive::getWheelSpeeds,
            new PIDConstants(1.5, 0, 0),
            drive::driveVoltage,
            null,
            true,
            drive);

        
        limelightInterface = new LimelightInterface();

        configureBindings(); 
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            drive.driveCommand(this::getLeftAxis, this::getRightAxis));
        myDriveForwardCommand.onTrue(new DriveForwardCommand(drive, 2));
        myTurnToAngleCommand.onTrue(new TurnToAngleCommand(drive, 90));
    }
    private double getLeftAxis () {
            return -deadband(leftJoystick.getY(),  0.1);
            
    }
    private double getRightAxis() {
            return -deadband(rightJoystick.getX(),  0.1);
    }

    private static double deadband(double value, double tolerance){
        if (Math.abs(value) < tolerance) return 0.0;
        
        return Math.copySign((value - tolerance) / (1.0 - tolerance), value); 
    }

    public Command getAutonomousCommand() {
        return autoBuilder.fullAuto(
            PathPlanner.loadPath("Raichu", new PathConstraints(3, 2.5)));

    }
}

    
    
