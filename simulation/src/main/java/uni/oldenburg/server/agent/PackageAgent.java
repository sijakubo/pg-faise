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
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.TickerBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.event.EventHelper;
import uni.oldenburg.shared.model.event.PackageAddedEvent;

/**
 * @author Matthias
 */
@SuppressWarnings("serial")
public class PackageAgent extends Agent {
	public final static String NAME = "PackageAgent";

	private Conveyor myConveyor;
	private Szenario mySzenario;
	
	private boolean hasPendingIncomingJob = false;
	private boolean hasPendingOutgoingJob = false;

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
		
		addBehaviour(new AddPackage(MessageType.ADD_PACKAGE));
		addBehaviour(new GetPackageCount(MessageType.GET_PACKAGE_COUNT));
		addBehaviour(new GetPendingJobStatus(MessageType.GET_PENDING_JOB_STATUS));
		addBehaviour(new SetDestination(MessageType.SET_DESTINATION));
		addBehaviour(new SetIncomingJobFlag(MessageType.SET_PENDING_INCOMING_STATUS));
		addBehaviour(new GetIncomingJobStatus(MessageType.GET_INCOMING_JOB_STATUS));
		addBehaviour(new GetOutgoingJobStatus(MessageType.GET_OUTGOING_JOB_STATUS));
		
		
		// am i am ramp?
		if (myConveyor instanceof ConveyorRamp) {
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// what ramp type am i?
			switch(myRampConveyor.getRampType()) {
				case ConveyorRamp.RAMP_ENTRANCE:
					addBehaviour(new SendEnquirePackageRequest(this, 1000));
					break;
				case ConveyorRamp.RAMP_EXIT:
					addBehaviour(new SendEnquirePackageRequest(this, 1000));
					addBehaviour(new JobAvailable(MessageType.DEMAND_PACKAGE)); // only for exit ramps?					
					break;
				case ConveyorRamp.RAMP_STOREAGE:
					addBehaviour(new PackageFoundInStorage(MessageType.FIND_PACKAGE_IN_STORAGE));
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
	
	private class GetPendingJobStatus extends CyclicReceiverBehaviour {
		protected GetPendingJobStatus(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage msgGetPendingJobStatus = new ACLMessage(MessageType.GET_PENDING_JOB_STATUS);
			msgGetPendingJobStatus.addUserDefinedParameter("pendingJob", (hasPendingIncomingJob || hasPendingOutgoingJob) ? "1" : "0");
			msgGetPendingJobStatus.addReceiver(msg.getSender());
			
			send(msgGetPendingJobStatus);
		}
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
	private class AddPackage extends CyclicReceiverBehaviour {
		protected AddPackage(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException {
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData myPackage = (PackageData) msg.getContentObject();
			
			currentAgent.lstPackage.add(myPackage);
			
			EventHelper.addEvent(new PackageAddedEvent(myConveyor.getID(), myPackage.getPackageID()));
   
			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + ": package "+ myPackage.getPackageID() + " added");
            
			hasPendingIncomingJob = false;
			
			ACLMessage msgCompleted = new ACLMessage(MessageType.ADD_PACKAGE_COMPLETED);
			msgCompleted.addReceiver(msg.getSender());
			send(msgCompleted);
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
	private class GetPackageCount extends CyclicReceiverBehaviour {
		protected GetPackageCount(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
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
	 * Got message:
	 * 		none [first behaviour to start sending enquires]
	 * Send message:
	 * 		RamOrderAgent::SendEquirePackageRequestRelay
	 * 
	 * sends message to own order agent to relay the ramp-enquiring
	 * 
     * @author Matthias
     */
	private class SendEnquirePackageRequest extends TickerBehaviour {
		public SendEnquirePackageRequest(Agent a, long period) {
			super(a, period);
		}

		protected void onTick() {
			ACLMessage enquireRampsMsg = new ACLMessage(MessageType.ENQUIRE_RAMPS_RELAY);
			ConveyorRamp myRampConveyor = (ConveyorRamp)myConveyor;
			
			// no package? -> no need to ask
			if (lstPackage.size() < 1)
				return;
			
			switch(myRampConveyor.getRampType())  {
				case ConveyorRamp.RAMP_ENTRANCE:
					// outgoing job already in progress? -> don't ask for now
					if (hasPendingOutgoingJob) return;
					
					enquireRampsMsg.addUserDefinedParameter("packageID", "" + lstPackage.get(0).getPackageID());
					break;
				case ConveyorRamp.RAMP_EXIT:
					// incoming job already in progress? -> don't ask for now
					if (hasPendingIncomingJob) return;
					
					int randomIndex = (int)(Math.random() * 100) % lstPackage.size();
					enquireRampsMsg.addUserDefinedParameter("packageID", "" + lstPackage.get(randomIndex).getPackageID());
					break;
				default:
					return;
			}
			
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
		protected JobAvailable(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
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
		protected PackageFoundInStorage(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
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
	
	private class SetDestination extends CyclicReceiverBehaviour {
		protected SetDestination(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			int destinationID = Integer.parseInt(msg.getUserDefinedParameter("destinationID"));
			
			lstPackage.get(0).setDestinationID(destinationID);
			
			hasPendingOutgoingJob = true;
			
			ACLMessage msgAnswer = new ACLMessage(MessageType.SET_DESTINATION_COMPLETED);
			msgAnswer.addReceiver(msg.getSender());
			send(msgAnswer);
		}		
	}
	
	private class SetIncomingJobFlag extends CyclicReceiverBehaviour {
		protected SetIncomingJobFlag(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			hasPendingIncomingJob = true;
			
			//logger.log(Level.INFO, "[SetIncomingJobFlag] Conveyor " + myConveyor.getID() + " incoming !!!");
			
			ACLMessage msgAnswer = new ACLMessage(MessageType.SET_JOB_FLAG_COMPLETED);
			msgAnswer.addReceiver(msg.getSender());
			send(msgAnswer);
		}
	}
	
	private class GetIncomingJobStatus extends CyclicReceiverBehaviour {
		protected GetIncomingJobStatus(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage msgAnswer = new ACLMessage(MessageType.GET_INCOMING_JOB_STATUS);
			msgAnswer.addUserDefinedParameter("status", hasPendingIncomingJob ? "1" : "0");
			msgAnswer.addReceiver(msg.getSender());
			send(msgAnswer);
		}		
	}
	
	private class GetOutgoingJobStatus extends CyclicReceiverBehaviour {
		protected GetOutgoingJobStatus(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage msgAnswer = new ACLMessage(MessageType.GET_OUTGOING_JOB_STATUS);
			msgAnswer.addUserDefinedParameter("status", hasPendingOutgoingJob ? "1" : "0");
			msgAnswer.addReceiver(msg.getSender());
			send(msgAnswer);
		}		
	}
}
