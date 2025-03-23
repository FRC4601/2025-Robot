package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Mobility extends SequentialCommandGroup {
    
    public Mobility(PathPlannerPath... paths) {

        addCommands(

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])

        );

    }

}
