package org.firstinspires.ftc.teamcode.teleops

import dev.nextftc.robot.*

import org.firstinspires.ftc.teamcode.BuckoBee

import dev.nextftc.robot.opmode.*

@NextTeleop(name = "My Teleop")
class MyTeleop(robot: BuckoBee) : NextOpMode(robot) {
	override fun periodic() {
		Telemetry.log("Status", "Running")
	}
	
	override fun start() {
		robot
	}
}