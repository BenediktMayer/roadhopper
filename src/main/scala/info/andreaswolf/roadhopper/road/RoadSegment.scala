package info.andreaswolf.roadhopper.road

import com.graphhopper.util.shapes.GHPoint3D

object RoadSegment {

	/**
	 * The earth’s radius as defined for WGS84.
	 */
	val R = 6371000

	/**
	 *
	 * @return A RoadSegment instance created from the coordinates
	 *
	 * Note that this uses an approximation for the length that treats the line segment as being orthogonal to the
	 * earth’s radius (Pythagorean approximation). This is generally considered safe for the short distances we have.
	 * <http://www.movable-type.co.uk/scripts/latlong.html>
	 */
	def fromCoordinates(lat1: Double, lon1: Double, lat2: Double, lon2: Double) : RoadSegment = {
		new RoadSegment(new GHPoint3D(lat1, lon1, 0.0), new GHPoint3D(lat2, lon2, 0.0))
	}

	def fromPoints(start: GHPoint3D, end: GHPoint3D): RoadSegment = {
		new RoadSegment(start, end)
	}

	/**
	 * Returns the length and orientation of the road segment. The returned length is slightly inaccurate, as
	 * the calculation does not take into account the bended earth surface
	 *
	 * @param start The start coordinate
	 * @param end The end coordinate
	 */
	protected def getLengthAndOrientation(start: GHPoint3D, end: GHPoint3D): (Double, Double) = {
		val phi1 = start.lat.toRadians
		val phi2 = end.lat.toRadians
		val deltaLambda = (end.lon - start.lon).toRadians
		val x = deltaLambda * Math.cos((phi1+phi2)/2)
		val y = phi2 - phi1
		val length = Math.sqrt(x*x + y*y) * R
		val orientation = Math.atan2(Math.sin(end.lon - start.lon) * Math.cos(end.lat), Math.cos(start.lat) * Math.sin(end.lat) -
			Math.sin(start.lat) * Math.cos(end.lat) * Math.cos(end.lon - start.lon))

		val normalizedOrientation = orientation match {
			case o if o < -Math.PI => o + (Math.PI * 2)
			// ensure that the interval is open at the right end
			case o if o % (Math.PI * 2) == Math.PI => -Math.PI
			case o if o >= Math.PI * 2 => (o - Math.PI * 2) % (Math.PI * 2)
			case o if o >= Math.PI => o - (Math.PI * 2)
			case o => o
		}

		(length, normalizedOrientation)
	}

}

/**
 *
 */
class RoadSegment(val start: GHPoint3D, val end: GHPoint3D) extends RoutePart {
	// length is slightly inaccurate as we use a simplified formula for calculating it
	lazy val (length, orientation) = RoadSegment.getLengthAndOrientation(start, end)

	/**
	 * Returns the angle necessary to get from this segment to the given segment.
	 *
	 * The return value is confined to an interval [-pi..pi). The resulting orientation is always absolute (i.e. in
	 * [-pi..pi)) and not relative to the start of the journey.
	 */
	def calculateNecessaryTurn(nextSegment: RoadSegment): Double = {
		nextSegment.orientation - orientation match {
			case x if x >= Math.PI * 2 => x % Math.PI
				// TODO this will not suffice if the value is greater than 3 pi/smaller than -3pi
				// a formula to fix this could look like this: (x + ((Math.abs(x) / (Math.PI * 2)).floor * Math.PI * 2))
			case x if x >= Math.PI => x - Math.PI * 2
			case x if x < -Math.PI => x + Math.PI * 2
			case x => x
		}
	}

	override def toString = f"RoadSegment($length%.2f, ${orientation.toDegrees}%.2f°)"
}
