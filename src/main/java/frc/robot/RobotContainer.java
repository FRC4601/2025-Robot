// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Autos.Mobility;
import frc.robot.commands.Autos.PathUtil.AutonConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentricFacingAngle robotdrive = new SwerveRequest.RobotCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController joystick = new CommandXboxController(0);
    public final static CommandXboxController xbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Smartdashboard choosers/data
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public static Arm m_arm = new Arm();
    public static Elevator m_elevator = new Elevator();
    public static Intake m_intake = new Intake();
    public static Vision m_vision = new Vision();


    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        configureBindings();


        //Set Default Commands
        
        m_arm.setDefaultCommand(new RunArm());
        m_elevator.setDefaultCommand(new RunElevator());
        m_intake.setDefaultCommand(new RunIntake());

        m_autoChooser = AutoBuilder.buildAutoChooser("Get Off Line Auto");


        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void configureBindings() {
        
        
        //Driver Controls (JOYSTICK)

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed *.5) // Drive forward with negative Y (forward)                  Add "-" For Blue Forward --- Remove "-" For Red Forward
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed *.5) // Drive left with negative X (left)                         Add "-" For Blue Forward --- Remove "-" For Red Forward
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)     Add "-" For Blue Forward --- Remove "-" For Red Forward
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY() * MaxSpeed *.5, -joystick.getLeftX()* MaxSpeed *.5))
            //robotdrive.withTargetDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))  //                          Remove "-" For Blue Forward ---  Add "-" For Red Forward
            //            .withVelocityX(joystick.getLeftY() * MaxSpeed *.5)                              //                          Add "-" For Blue Forward --- Remove "-" For Red Forward
            //            .withVelocityY(joystick.getLeftX() * MaxSpeed *.5)                              //                          Add "-" For Blue Forward --- Remove "-" For Red Forward
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on start button press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        //Co-Pilot Controls (XBOX)

    }



    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        return m_autoChooser.getSelected();
    
    }
}
