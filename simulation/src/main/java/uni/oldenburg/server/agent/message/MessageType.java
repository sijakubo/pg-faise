package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   = 0x00; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   = 0x01; // RampPlattform -> JobAgent
	public final static int START_AUCTION              = 0x02; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION            = 0x03; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	   = 0x04; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   = 0x05; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   = 0x06; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   = 0x07; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   = 0x08; // PackageAgent
	public final static int REMOVE_PACKAGE		 	   = 0x09; // PackageAgent
	public final static int ASSIGN_JOB				   = 0x0A; // JobAgent
	public final static int ASSIGN_VEHICLE_FOR_PACKAGE = 0x0B; // RampRouting @ VehicleRouting
	public final static int SEARCH_FOR_PACKAGE = 0x0C; //PackageAgent (Exit) @ OrderAgent (EXIT)
	public final static int ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS = 0x0D; // OrderAgent(Exit) @ OrderAgent (Storage)
	public final static int CHECK_IF_PACKAGE_IS_STORED = 0xE; // OrderAgent(Storage) @ PackageAgent (Storage)
	public final static int ANSWER_IF_PACKAGE_IS_CONTAINED = 0xE; // PackageAgent (Storage)@ OrderAgent(Storage) 	
	public final static int GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT = 0xF; // OrderAgent (Storage) @ OrderAgent (Exit)
	//public final static int SET_PACKAGE_RESERVED = 0x10; // Orderagent (Exit)
}
