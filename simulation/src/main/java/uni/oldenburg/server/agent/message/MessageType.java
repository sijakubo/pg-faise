package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   				= 0; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   				= 1; // RampPlattform -> JobAgent
	public final static int START_AUCTION              				= 2; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION            				= 3; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	   				= 4; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   				= 5; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   				= 6; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   				= 7; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   				= 8; // PackageAgent
	public final static int REMOVE_PACKAGE		 	   				= 9; // PackageAgent
	public final static int ASSIGN_JOB				   				= 10; // JobAgent
	public final static int ENQUIRE_RAMPS_WITHOUT_ENTRANCE			= 11; // Entrance/Exit PackageAgent -> Own OrderAgent
	public final static int ENQUIRE_RAMPS_RELAY						= 12; // Own OrderAgent -> Storage/Exit OrderAgent
	public final static int ENQUIRE_RAMP_RESPONSE					= 13;
	public final static int DEMAND_PACKAGE							= 14;
	public final static int ENQUIRE_RAMPS_STORAGE					= 15;
	public final static int FIND_PACKAGE_IN_STORAGE					= 16;
}
