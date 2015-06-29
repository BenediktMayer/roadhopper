package info.andreaswolf.roadhopper.simulation

import akka.actor.{Actor, ActorRef}
import info.andreaswolf.roadhopper.road.{RoadBendEvaluator, RoadSegment}

/**
 * A vehicle driver, responsible for steering the vehicle.
 */
class Driver {

	def changeVehicleInput(currentStep: SimulationStep): DriverInput = {
		// TODO adjust steering wheel
		new DriverInput(0.0)
	}

}


class DriverActor(val timer: ActorRef, val vehicle: ActorRef, val journey: ActorRef) extends Actor {
	var steps = 0
	protected var currentTime = 0

	val bendEvaluator = new RoadBendEvaluator

	override def receive: Receive = {
		case Start() =>
			println("Driver starting")
			vehicle ! Accelerate(1.0)
			timer ! ScheduleRequest(40)

		case Step(time) =>
			currentTime = time
			vehicle ! RequestVehicleStatus()

		case RoadAhead(time, roadParts) =>
			if (currentTime % 2000 == 0) {
				println(roadParts.length + " road segment immediately ahead; " + currentTime)
				println(bendEvaluator.findBend(roadParts.collect { case b:RoadSegment => b }))
			}

			timer ! ScheduleRequest(currentTime + 40)

		case VehicleStatus(time, state, travelledDistance) =>
			if (travelledDistance > 10000) {
				if (state.speed < -0.25) {
					vehicle ! SetAcceleration(1.0)
				} else if (state.speed > 0.25) {
					vehicle ! SetAcceleration(-1.0)
				} else {
					vehicle ! SetAcceleration(0.0)
					timer ! StopSimulation()
				}
			}
			journey ! RequestRoadAhead(travelledDistance.toInt)
	}
}
