package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Job;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampPlattformAgent extends Agent {
	public final static String NAME = "RampPlattformAgent";
	
	private int conveyorID = 0;
	private Szenario szenario;
	private int rampType = 0;
	private int packageCountMax = 0;
	private Logger logger = Logger.getLogger(RampPlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenario = (Szenario) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
			//rampType = ((ConveyorRamp)myConveyor).getRampType();
			//packageCountMax = myConveyor.getPackageCountMax();
		}
		
		addBehaviour(new SendRampInfoBehaviour());
		addBehaviour(new IsPackageSpaceAvailableBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenario.getId());
		AgentHelper.registerAgent(szenario.getId(), this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		JobAgent::RequestRampInfosBehaviour
	 * Send message: 
	 * 		JobAgent::ReceiveRampInfosBehaviour
	 * 
	 * send information about the ramp type
	 * 
     * @author Matthias
     */
	private class SendRampInfoBehaviour extends OneShotBehaviour {	
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;
			
			// get ramp info request
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.REQUEST_RAMP_INFO);			
			ACLMessage msg = myAgent.blockingReceive(mt);
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- REQUEST_RAMP_INFO");
			
			// send ramp info data
			ACLMessage msgReply = new ACLMessage(MessageType.SEND_RAMP_INFO);
			msgReply.addUserDefinedParameter("rampType", "" + currentAgent.rampType);
			msgReply.addReceiver(msg.getSender());
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> SEND_RAMP_INFO");
			
			send(msgReply);
		}
	}

	/**
	 * Got message:
	 * 		JobAgent::DistributeJobBehaviour
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 			[after ramp was chosen]
	 * Send message:
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 			[ask ramps for item count]
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 		PackageAgent::AddPackageBehaviour
	 * 			[only if current ramp got the job]
	 * 
	 * send message if space is available for a package on this ramp
	 * and wait for a response if a new space should be "reserved" or not
	 * 
     * @author Matthias
     */
	private class IsPackageSpaceAvailableBehaviour extends CyclicBehaviour {
		int step = 0;
		int packageCount = 0;
		
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;

			if (step == 0) {
				// get ramp space request
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_SPACE_AVAILABLE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- PACKAGE_SPACE_AVAILABLE");
					
					try {
						// get package count from packageagent
						// request
						ACLMessage msgPackageReq = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
						AgentHelper.addReceiver(msgPackageReq, myAgent, PackageAgent.NAME, currentAgent.conveyorID,
                        currentAgent.szenario.getId());
						
					   logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_PACKAGE_COUNT");
						
						send(msgPackageReq);
						
						// response
						mt = MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT);
						ACLMessage msgPackageRes = myAgent.blockingReceive(mt);
						
						logger.log(Level.INFO, myAgent.getLocalName() + " <- GET_PACKAGE_COUNT");
						
						packageCount = Integer.parseInt(msgPackageRes.getUserDefinedParameter("package_count"));
						int packageCountMax = currentAgent.packageCountMax;
						String isSpaceAvailable = packageCount < packageCountMax ? "1" : "0";
						
						// always space available for outgoing ramps
						if (currentAgent.rampType == ConveyorRamp.RAMP_EXIT)
							isSpaceAvailable = "1";
						
						// send ramp space info
						ACLMessage msgReply = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
						msgReply.addUserDefinedParameter("space_available", isSpaceAvailable);


						String enquiringRampConveyorId = msg.getUserDefinedParameter("enquiring_ramp_conveyor_id");
						if (enquiringRampConveyorId != null) {
							msgReply.addUserDefinedParameter("enquiring_ramp_conveyor_id", enquiringRampConveyorId);
						}

						//Job or PackageData
						msgReply.setContentObject(msg.getContentObject());
						msgReply.addReceiver(msg.getSender());
						
						if(Debugging.showInfoMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " -> PACKAGE_SPACE_AVAILABLE");

						if (msg.getUserDefinedParameter("information_message_no_step") == null) {
							++step;
						}

						send(msgReply);
					} catch (UnreadableException e) {
						e.printStackTrace();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				else {
					block();
				}
			}
			else if (step == 1) {
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.RESERVE_SPACE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- RESERVE_SPACE");
					
					String target = msg.getUserDefinedParameter("RampAID");
					
					// reserve/add space in ramp if target matches 
					if (target.compareTo(myAgent.getAID().toString()) == 0) {
						ACLMessage msgAddPackage = new ACLMessage(MessageType.ADD_PACKAGE);
						AgentHelper.addReceiver(msgAddPackage, currentAgent, PackageAgent.NAME, conveyorID, szenario.getId());
						
						++packageCount;
							
						try {
							Job assigningJob = (Job)msg.getContentObject();
							
							// create packagedata from job infos
							PackageData myPackageData = new PackageData(assigningJob.getPackageId(), assigningJob.getDestinationId());
							
							msgAddPackage.setContentObject(myPackageData);
							send(msgAddPackage);
						}
						catch (IOException e) {
							e.printStackTrace();
						}
						catch (UnreadableException e) {
							e.printStackTrace();
						}
						
						if(Debugging.showInfoMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " - space reserved: " + packageCount + "/" + currentAgent.packageCountMax);
					}
					
					step = 0;
				}
				else {
					block();
				}
			}
		}
	}
}
