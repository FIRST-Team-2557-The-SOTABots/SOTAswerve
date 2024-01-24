package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class DriveCommand extends Command {
    private SwerveDrive mSwerveDrive;
    private DoubleSupplier frwdSup;
    private DoubleSupplier strfSup;
    private DoubleSupplier rttnSup;

    public DriveCommand(SwerveDrive swerveDrive, DoubleSupplier frwdSup, DoubleSupplier strfSup, DoubleSupplier rttnSup) {
        this.mSwerveDrive = swerveDrive;
        this.frwdSup = frwdSup;
        this.strfSup = strfSup;
        this.rttnSup = rttnSup;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double frwd = Math.signum(frwdSup.getAsDouble()) * frwdSup.getAsDouble() * frwdSup.getAsDouble();
        double strf = Math.signum(strfSup.getAsDouble()) * strfSup.getAsDouble() * strfSup.getAsDouble();
        double rttn = Math.signum(rttnSup.getAsDouble()) * rttnSup.getAsDouble() * rttnSup.getAsDouble();

        mSwerveDrive.drive(frwd, strf, -rttn);
    }

}
