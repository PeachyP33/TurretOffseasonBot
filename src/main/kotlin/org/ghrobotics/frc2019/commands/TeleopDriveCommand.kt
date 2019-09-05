package org.ghrobotics.frc2019.commands

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kA

class TeleopDriveCommand : FalconCommand(Drivetrain) {
    override fun execute() {
        Drivetrain.curvatureDrive(-xSource(), cSource(), qSource())
    }

    companion object {
        private val xSource = Controls.driverController.getY(GenericHID.Hand.kLeft)
        private val cSource = Controls.driverController.getX(GenericHID.Hand.kLeft)
        private val qSource = Controls.driverController.getRawButton(kA)
    }
}