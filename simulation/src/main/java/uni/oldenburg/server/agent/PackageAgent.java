package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.helper.EventHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.TickerBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.event.PackageAddedEvent;

/**
 * @author Matthias
 */
@SuppressWarnings("serial")
public class PackageAgent extends Agent {
	public final static String NAME = "PackageAgent";

	private Conveyor myConveyor;
	private Szenario mySzenario;
	
	private boolean hasPendingJob = false; 

	private List<PackageData> lstPackage = new ArrayList<PackageData>();

	private Logger logger = Logger.getLogger(PackageAgent.class);

   /**
	 * @author Matthias, sijakubo
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (Conveyor) args[1];
		}
		
		addBehaviour(new AddPackageBehaviour(MessageTemplate.MatchPerformative(MessageType.ADD_PACKAGE)));
		addBehaviour(new GetPackageCountBehaviour(MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT)));
		addBehaviour(new RemovePackageBehaviour(MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE)));
		addBehaviour(new JobAvailable(MessageTemplate.MatchPerformative(MessageType.DEMAND_PACKAGE))); // only for exit ramps?
		
		// am i am ramp?
		if (myConveyor instanceof ConveyorRamp) {
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// what ramp type am i?
			switch(myRampConveyor.getRampType()) {
				case ConveyorRamp.RAMP_ENTRANCE:
					addBehaviour(new SendEquirePackageRequest(this, 1000));
					break;
				case ConveyorRamp.RAMP_EXIT:
					addBehaviour(new SendEquirePackageRequest(this, 1000));
					break;
				case ConveyorRamp.RAMP_STOREAGE:
					addBehaviour(new PackageFoundInStorage(MessageTemplate.MatchPerformative(MessageType.FIND_PACKAGE_IN_STORAGE)));
					break;
			}
		}
         
		String nickname = AgentHelper.getUniqueNickname(PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
 		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

 		if (Debugging.showAgentStartupMessages)
 			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		RampPlattformAgent::IsPackageSpaceAvailableBehaviour
	 * 			[from chosen ramp]
	 * Send message:
	 * 		none
	 * 
	 * add package to this agent
	 * 
	 * @author Matthias, Raschid
	 */
	private class AddPackageBehaviour extends CyclicReceiverBehaviour {
		protected AddPackageBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException {
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData myPackage = (PackageData) msg.getContentObject();
			
			currentAgent.lstPackage.add(myPackage);
			
			EventHelper.addEvent(new PackageAddedEvent(myConveyor.getID()));
   
			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package "+ myPackage.getPackageID() + " added");
            
		}
	}

	/**
	 * Got message:
	 * 		RampPlattformAgent::IsPackageSpaceAvailableBehaviour
	 * Send message:
	 * 		RampPlattformAgent::IsPackageSpaceAvailableBehaviour
	 * 
	 * give into about current package count
	 * 
	 * @author Matthias
	 */
	private class GetPackageCountBehaviour extends CyclicReceiverBehaviour {
		protected GetPackageCountBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) {
			PackageAgent currentAgent = (PackageAgent) myAgent;

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- GET_PACKAGE_COUNT");

			ACLMessage msgReply = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
			msgReply.addUserDefinedParameter("package_count", "" + currentAgent.lstPackage.size());
			msgReply.addReceiver(msg.getSender());

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_PACKAGE_COUNT");

			send(msgReply);
		}
	}

	/**
	 * remove package from agent
	 * 
	 * @author Matthias
	 */
	private class RemovePackageBehaviour extends CyclicReceiverBehaviour {
		public RemovePackageBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException {
			PackageAgent currentAgent = (PackageAgent) myAgent;

			PackageData myPackage = (PackageData) msg.getContentObject();
			currentAgent.lstPackage.remove(myPackage);

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package " + myPackage.getPackageID() + " removed");
			
		}
	}
	
	
	/**
	 * Got message:
	 * 		none [first behaviour to start sending enquires]
	 * Send message:
	 * 		RamOrderAgent::SendEquirePackageRequestRelay
	 * 
	 * sends message to own order agent to relay the ramp-enquiring
	 * 
     * @author Matthias
     */
	private class SendEquirePackageRequest extends TickerBehaviour {
		public SendEquirePackageRequest(Agent a, long period) {
			super(a, period);
		}

		protected void onTick() {
			ACLMessage enquireRampsMsg = new ACLMessage(MessageType.ENQUIRE_RAMPS_RELAY);
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// job already in progress? -> don't ask
			if (hasPendingJob)
				return;
			
			// no package? -> no need to ask
			if (lstPackage.size() < 1)
				return;
			
			switch(myRampConveyor.getRampType())  {
				case ConveyorRamp.RAMP_ENTRANCE:
					enquireRampsMsg.addUserDefinedParameter("packageID", "" + lstPackage.get(0).getPackageID());
					break;
				case ConveyorRamp.RAMP_EXIT:
					int randomIndex = (int)(Math.random() * 100) % lstPackage.size();
					enquireRampsMsg.addUserDefinedParameter("packageID", "" + lstPackage.get(randomIndex).getPackageID());
					break;
				default:
					return;
			}
			
			enquireRampsMsg.addUserDefinedParameter("requestingRampType", "" + myRampConveyor.getRampType());
			
			// send to own orderagent
			AgentHelper.addReceiver(enquireRampsMsg, myAgent, RampOrderAgent.NAME, myConveyor.getID(), mySzenario.getId());
			send(enquireRampsMsg);
		}			
	}
	
	/**
	 * Got message:
	 * 		PackageAgent::SendEquirePackageRequest
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * Send message:
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 
	 * enquires ramp if they want to get a package
	 * 
     * @author Matthias
     */
	private class JobAvailable extends CyclicReceiverBehaviour {
		protected JobAvailable(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			boolean demanding = false;
			
			int packageID = Integer.parseInt(msg.getUserDefinedParameter("packageID"));
			
			for(PackageData myPackage : lstPackage) {
				if (myPackage.getPackageID() == packageID) {
					demanding = true;
					break;
				}
			}
			
			// response "demanding" status
			ACLMessage msgDemandPackageResponse = new ACLMessage(MessageType.DEMAND_PACKAGE);
			msgDemandPackageResponse.addUserDefinedParameter("demanding", demanding == true ? "1" : "0");
			msgDemandPackageResponse.addReceiver(msg.getSender());
			send(msgDemandPackageResponse);
		}
	}
	
	/**
	 * Got message:
	 * 		RampOrderAgent::GetEnquirePackageRequest
	 * Send message:
	 * 		RampOrderAgent::GetEnquirePackageRequest
	 * 
	 * sends info is package is found in first position in the packagelist
	 * 
     * @author Matthias
     */
	private class PackageFoundInStorage extends CyclicReceiverBehaviour {
		protected PackageFoundInStorage(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			boolean demanding = false;
			
			int packageID = Integer.parseInt(msg.getUserDefinedParameter("packageID"));
			
			if (lstPackage.size() > 0) {
				if (lstPackage.get(0).getPackageID() == packageID)
					demanding = true;	
			}
			
			// response "demanding" status
			ACLMessage msgDemandPackageResponse = new ACLMessage(MessageType.FIND_PACKAGE_IN_STORAGE);
			msgDemandPackageResponse.addUserDefinedParameter("demanding", demanding == true ? "1" : "0");
			msgDemandPackageResponse.addReceiver(msg.getSender());
			send(msgDemandPackageResponse);
		}
	}
}
