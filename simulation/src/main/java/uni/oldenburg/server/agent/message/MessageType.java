package uni.oldenburg.server.agent.message;

/**
 * @author Christopher Matthias
 */	
public class MessageType {
	public final static int REQUEST_RAMP_INFO 		   = 0; // JobAgent -> RampPlattform
	public final static int SEND_RAMP_INFO 			   = 1; // RampPlattform -> JobAgent
	public final static int START_AUCTION              = 2; // RampRouting @ VehicleRouting
	public final static int SEND_ESTIMATION            = 3; // VehicleRouting @ RampRouting
	public final static int PACKAGE_SPACE_AVAILABLE	   = 4; // JobAgent <-> RampPlattform
	public final static int SEND_JOB				   = 5; // JobAgent -> JobAgent
	public final static int RESERVE_SPACE			   = 6; // JobAgent -> RampPlattform
	public final static int ADD_PACKAGE				   = 7; // PackageAgent
	public final static int GET_PACKAGE_COUNT	 	   = 8; // PackageAgent
	public final static int REMOVE_PACKAGE		 	   = 9; // PackageAgent
	public final static int ASSIGN_JOB				   = 10; // JobAgent
	public final static int ASSIGN_VEHICLE_FOR_PACKAGE = 11; // RampRouting @ VehicleRouting
	public final static int SEARCH_FOR_PACKAGE = 12; //PackageAgent (Exit) @ OrderAgent (EXIT)
	public final static int ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS = 13; // OrderAgent(Exit) @ OrderAgent (Storage)
	public final static int CHECK_IF_PACKAGE_IS_STORED = 14; // OrderAgent(Storage) @ PackageAgent (Storage)
	public final static int ANSWER_IF_PACKAGE_IS_CONTAINED = 15; // PackageAgent (Storage)@ OrderAgent(Storage)
	public final static int GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT = 16; // OrderAgent (Storage) @ OrderAgent (Exit)
	public final static int SET_PACKAGE_RESERVED = 17; // Orderagent (Exit)@ PackageAgent (Exit)
	public final static int PACKAGE_RESERVATION = 18; //  PackageAgent (Exit)
	public final static int SET_PACKAGE_DESTINATION_STORAGE = 19; //  PackageAgent (Exit)
	public final static int INITIALIZE_START_AUCTION_BEHAVIOUR = 20; //  PackageAgent (Entry/Storage) @ Routingagent

   public static final int START_RAMP_SEARCH_FOR_PACKAGE = 21;
   public static final int ENQUIRE_OUTGOING_RAMP = 22;

   public static final int START_RAMP_EXIT_PACKAGE_ENQUIRE = 23;
   public static final int PACKAGE_IS_NEEDED_FROM_RAMP_EXIT = 24;

   public static final int GET_PACKAGE_FROM_SOURCE = 25;
   public static final int GIVE_PACKAGE = 26;
   public static final int REMOVE_PACKAGE_AND_ANSWER = 27;
   public static final int PACKAGE_REMOVED = 28;
   public static final int ANSWER_BOT = 29;
   public static final int BOT_ADD_PACKAGE = 30;
   public static final int BOT_GO_TO_DESTINATION = 31;
   public static final int BOT_TARGET_ACHIEVED = 32;
   public static final int CAN_TAKE_PACKAGE = 33;
   public static final int BOT_REMOVE_PACKAGE = 34;
   public static final int BOT_REMOVED_PACKAGE = 35;
   public static final int RAMP_TAKE_PACKAGE = 36;
   public static final int SET_BOT_UNRESERVED = 37;

   public static final int START_EXIT_RAMP_SEARCH_FOR_PACKAGE = 38;
   public static final int END_EXIT_RAMP_SEARCH_FOR_PACKAGE = 39;

   public static final int ENQUIRE_EXIT_RAMP = 40;
   public static final int ENQUIRE_STORAGE_RAMP = 41;

   public static final int CHECK_IF_PACKAGE_IS_NEEDED = 42;
   public static final int PACKAGE_IS_NEEDED = 43;

   public static final int PACKAGE_IS_NEEDED_FROM_EXIT_RAMP = 44;
   public static final int PACKAGE_IS_STORABLE_FROM_STORAGE_RAMP = 45;

   public static final int RESERVE_PACKAGE_SLOT_ON_RAMP = 46;
   public static final int ASSIGN_PACKAGE_DESTINATION = 47;

   public static final int SET_PACKAGE_RESERVED_FOR_STORAGE_RAMP = 48;

   public static final int START_STORAGE_RAMP_SEARCH_FOR_PACKAGE = 49 ;
   public static final int END_STORAGE_RAMP_SEARCH_FOR_PACKAGE = 50;

   public static final int RAMP_FREE_FOR_PACKAGE_ENQUIRE = 51;
}
