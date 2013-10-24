package parkingRobot;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;


/**
 * interface for the main module 'Navigation', providing methods for estimation of the robot position, its heading angle
 * and information about detected parking slots.
 * <p>
 * It contains the subclass {@link ParkingSlot} to provide a reasonable data structure to the cooperating classes containing
 * all relevant information for one parking slot detected by the Navigation class.
 * 
 * @author IfA
 */
public interface INavigation {
	
	// Inputs 	
	
	/**
	 * set the map of the line the robot follows while searching for parking slots
	 * 
	 * @param map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	public void setMap(Line[] map);
	
	/**
	 * switches parking slot detection on or off. While a parking maneuver active parking slot detection can lead to unexpected
	 * parking slot initializations or measurements. The detection should only be activated while the robot is in an explicit
	 * parking slot search state.
	 * 
	 * @param isOn indicates if parking slot detection should be switched on (true) or off (false)
	 */
	public void setDetectionState(boolean isOn);
	
	
	// Class control
	
	/**
	 * initiates a new and actual computation/detection of all relevant Navigation members, especially robot pose and parking slots (last, if their detection is activated)
	 */
	public void updateNavigation();
	
	
	// Outputs
	
	/**
	 * returns bundled current X and Y location and heading angle phi
	 * 
	 * @return pose class containing pose at time of last computation 
	 */
	public Pose getPose();

	/**
	 * returns references to all known parking slot classes with information about ID, status, start position and end position
	 * 
	 * @return array of all known parking space class references
	 */
	public ParkingSlot[] getParkingSlots();
	
	
	// Subclasses
	
	/**
	 * contains all necessary information about one parking slot. This is the slot status (whether it is/is not satisfying or has
	 * to be scanned again), its ID, the position of the slot begin front boundary and the position of the slot back boundary.
	 * 
	 * @author IfA
	 */
	public class ParkingSlot {		
		// members
		
		/**
		 * states the parking slot can be in
		 */
		public enum ParkingSlotStatus{
			/**
			 * indicates robot could fit in parking slot
			 */
			GOOD,
			/**
			 * indicates that parking slot space is to small
			 */
			BAD,
			/**
			 * indicates that measurement was not satisfying yet
			 */
			RESCAN
		}
		
		/**
		 * stores the parking slot ID that is the number of recent detected parking spaces incremented by one. When delegating the
		 * robot to a special parking slot a distinction between the spaces is necessary and given by identification.  
		 */
		int ID;
		/**
		 * position of parking slot back boundary, robot passes it first then the front boundary
		 */
		Point backBoundaryPosition = null;
		/**
		 * position of parking slot front boundary, robot passes it second after the back boundary
		 */
		Point frontBoundaryPosition   = null;
		
		/**
		 * characterization of the parking slot measurement: GOOD - robot could fit in, BAD - space to small,
		 * RESCAN - measurement not satisfying, has to be scanned again
		 */
		private ParkingSlotStatus status = ParkingSlotStatus.RESCAN;
		
		
		// Constructor
		
		/**
		 * generate a ParkingSlot object without initialization of data members.
		 */
		ParkingSlot(int ID){
			this.ID = ID;
		}
		
		/**
		 * generate a ParkingSlot object and initialize all data members.
		 * 
		 * @param ID					stores the parking slot ID that is the number of recent detected parking spaces incremented by one
		 * @param backBoundaryPosition	position of parking slot back boundary, robot passes it first then the front boundary
		 * @param frontBoundaryPosition	position of parking slot front boundary, robot passes it second after the back boundary
		 * @param slotStatus			characterization of the parking slot measurement
		 */
		public ParkingSlot(int ID, Point backBoundaryPosition, Point frontBoundaryPosition, ParkingSlotStatus slotStatus){
			this.ID     				= ID;
			this.backBoundaryPosition  	= backBoundaryPosition;
			this.frontBoundaryPosition	= frontBoundaryPosition;
			this.setStatus(slotStatus);		
		}
		
		// Set methods
		
		/**
		 * @param ID stores the parking slot ID that is the number of recent detected parking spaces incremented by one
		 */
		public void setID(int ID){
			this.ID = ID;
		}
		/**
		 * @param backBoundaryPosition position of parking slot back boundary, robot passes it first then the front boundary
		 */
		public void setBackBoundaryPosition(Point backBoundaryPosition){
			this.backBoundaryPosition  = backBoundaryPosition;
		}
		/**
		 * @param frontBoundaryPosition position of parking slot front boundary, robot passes it second after the back boundary
		 */
		public void setFrontBoundaryPosition(Point frontBoundaryPosition){
			this.frontBoundaryPosition = frontBoundaryPosition;
		}
		/**
		 * @param slotStatus characterization of the parking slot measurement
		 */
		public void setParkingSlotStatus(ParkingSlotStatus slotStatus){
			this.setStatus(slotStatus);
		}
		
		
		// Get methods
		
		/**
		 * @return the parking slot ID that is the number of recent detected parking spaces incremented by one
		 */
		public int getID(){
			return ID;
		}
		/**
		 * @return backBoundaryPosition position of parking slot back boundary, robot passes it first then the front boundary
		 */
		public Point getBackBoundaryPosition(){
			return backBoundaryPosition;
		}
		/**
		 * @return frontBoundaryPosition position of parking slot front boundary, robot passes it second after the back boundary
		 */
		public Point getFrontBoundaryPosition(){
			return frontBoundaryPosition;
		}

		/**
		 * @return status characterization of the parking slot status
		 */
		public ParkingSlotStatus getStatus() {
			return status;
		}
		
		/**
		 * set parking slot status
		 * 
		 * @param status status of parking slot
		 */
		public void setStatus(ParkingSlotStatus status) {
			this.status = status;
		}
	}	
}