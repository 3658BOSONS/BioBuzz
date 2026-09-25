package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.robot.NextRobot
import dev.nextftc.robot.drive.mecanumDrive
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.drive.Drivetrain

class BuckoBee : NextRobot {
	val driveTrain: Drivetrain = Drivetrain()
	
	fun startDrive(gamepad1: Gamepad): Unit {
		mecanumDrive(
			this.driveTrain.frontLeftMotor,
			this.driveTrain.frontRightMotor,
			this.driveTrain.backLeftMotor,
			this.driveTrain.backRightMotor,
			gamepad1
		).schedule()
	}
	
	override val mechanisms = setOf(this.driveTrain); \
}