package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.drive.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.util.*

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.hardware.lynx.LynxModule

import com.pedropathing.ivy.*

import dev.nextftc.control.drive.MecanumKinematics
import dev.nextftc.robot.NextRobot
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.drive.mecanumDrive
import dev.nextftc.robot.triggers.CommandGamepad

class BuckoBee : NextRobot {
	val driveTrain: Drivetrain = Drivetrain()
	
	val outtake = Outtake()
	val intake = Intake()
	
	
	val status: StatusLight = StatusLight()
	
	val rc = RobotController
	
	fun startDrive(gamepad1: Gamepad, strafeCompensation: Double): Unit {
		mecanumDrive(
			this.driveTrain.frontLeftMotor,
			this.driveTrain.frontRightMotor,
			this.driveTrain.backLeftMotor,
			this.driveTrain.backRightMotor,
			gamepad1,
			kinematics = MecanumKinematics(strafeCompensation = strafeCompensation)
		).schedule()
		
		this.status.set(Status.Active)
	}
	
	fun startLauncherControls(gamepad2: Gamepad): Unit {
		val commandGamepad = CommandGamepad(gamepad2)
		
		commandGamepad.dpadUp.onTrue(this.outtake.up())
		commandGamepad.dpadDown.onTrue(this.outtake.down())
		
		commandGamepad.b.onTrue(this.outtake.grab())
		commandGamepad.y.onFalse(this.outtake.drop())
		
		commandGamepad.rightTrigger.isOver(0.1)
			.whileTrue(
				this.outtake.spin { commandGamepad.rightTrigger.value }
			)
			.onFalse(this.outtake.stop())
		
		
		commandGamepad.leftTrigger.isOver(0.1)
			.whileTrue(
				this.intake.spin { commandGamepad.leftTrigger.value }
			)
			.onFalse(this.intake.stop())
	}
//	override val mechanisms = setOf(this.driveTrain);
}