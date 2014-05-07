package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.Priority;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageDestinationData;
import uni.oldenburg.server.agent.data.EnquiredPackageData;
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

/**
 * @author Matthias
 */
@SuppressWarnings("serial")
public class PackageAgent extends Agent {
	public final static String NAME = "PackageAgent";

	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = -1;// -1 represents a Vehicle

	private List<PackageData> lstPackage = new ArrayList<PackageData>();

	private Logger logger = Logger.getLogger(PackageAgent.class);

	/**
	 * @author Matthias
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];

			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
			if (myConveyor instanceof ConveyorRamp) {
				rampType = ((ConveyorRamp) myConveyor).getRampType();
			}

		}

        	addBehaviour(new AddPackageBehaviour(
    				MessageTemplate.MatchPerformative(MessageType.ADD_PACKAGE)));
    
		
		addBehaviour(new GetPackageCountBehaviour(
				MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT)));
		addBehaviour(new RemovePackageBehaviour(
				MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE)));
        addBehaviour(new AssignDestinationToPackageBehaviour(
            MessageTemplate.MatchPerformative(MessageType.ASSIGN_PACKAGE_DESTINATION)));
        addBehaviour(new RemovePackageAndAnswerBehaviour(
				MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE_AND_ANSWER)));

		// If it is an Exit, than add the Behaviour, which is used for
		// requesting an Package for an existing Job
		if (rampType == ConveyorRamp.RAMP_EXIT) {
			addBehaviour(new SearchForPackageBehaviour(this, 3000));
			addBehaviour(new PackageReservationBehaviour(
					MessageTemplate.MatchPerformative(MessageType.SET_PACKAGE_RESERVED)));
         addBehaviour(new PackageNeedInformationBehaviour(MessageType.CHECK_IF_PACKAGE_IS_NEEDED));
		}

		// If it is an Storage it should answer the request from its own
		// Orderagent, who was asked by an exit, and check if a
		// Package for the requested Package id is contained
		if (rampType == ConveyorRamp.RAMP_STOREAGE) {
			addBehaviour(new AnswerIfPackageIsContainedBehaviour(MessageTemplate.MatchPerformative(MessageType.CHECK_IF_PACKAGE_IS_STORED)));
			addBehaviour (new SetPackageDestinationBehaviour(MessageTemplate.MatchPerformative(MessageType.SET_PACKAGE_DESTINATION_STORAGE)));
			addBehaviour (new InitializeAuctionStartBehaviour(this,3000));
			
		}
		
		if(rampType==ConveyorRamp.RAMP_ENTRANCE){
			addBehaviour (new InitializeAuctionStartBehaviour(this,3000));
		}
		
		if(rampType==-1){//If it is an Vehicle
			addBehaviour(new BotAddPackageBehaviour(MessageTemplate.MatchPerformative(MessageType.BOT_ADD_PACKAGE)));
			addBehaviour(new BotRemovePackageBehaviour(MessageTemplate.MatchPerformative(MessageType.BOT_REMOVE_PACKAGE)));			
		}
		

		String nickname = AgentHelper.getUniqueNickname(PackageAgent.NAME,
				conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");

	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	/**
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
            //If it is an Exit the Package can be removed immediately , The package is thrown down
			if(rampType==ConveyorRamp.RAMP_EXIT){
				
            	for(int i=0;i<currentAgent.lstPackage.size();i++){
            		PackageData dummy=currentAgent.lstPackage.get(i);
            		if(dummy.getPackageID()==myPackage.getPackageID()){
            			currentAgent.lstPackage.remove(i);
            			break;
            		}
            	}
            }else {
			   currentAgent.lstPackage.add(myPackage);
   
			    if (Debugging.showPackageMessages)
				    logger.log(Level.INFO, myAgent.getLocalName() + ": package "+ myPackage.getPackageID() + " added");
            }
		}
	}

	/**
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
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- GET_PACKAGE_COUNT");

			ACLMessage msgReply = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
			msgReply.addUserDefinedParameter("package_count", ""+ currentAgent.lstPackage.size());
			msgReply.addReceiver(msg.getSender());

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " -> GET_PACKAGE_COUNT");

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
				logger.log(Level.INFO, myAgent.getLocalName() + ": package "
						+ myPackage.getPackageID() + " removed");
		}
	}

	/**
	 * Behaviour (Exit Behaviour) should cyclically choose a Job from the List
	 * and ask his Orderagent to ask the Orderagents from the Exits if there is
	 * a Package, which belongs to the Job
	 * 
	 * @author Raschid
	 */
	private class SearchForPackageBehaviour extends TickerBehaviour {

		public SearchForPackageBehaviour(Agent a, long period) {
			super(a, period);

		}

		@Override
		protected void onTick() {
			PackageAgent currentAgent = (PackageAgent) myAgent;

			// Only if Jobs exists, there should be made a request
			int sizePackageList = currentAgent.lstPackage.size();
			if (sizePackageList >= 1) {
				// Choose a Package randomly. To do that, you have to create a
				// random Value depending on the size of the package list
				int index = (int) ((Math.random() * 10) % sizePackageList);
				/*
				 * if (sizePackageList == 1) { index = 0; } else if
				 * (sizePackageList == 2) { Random random = new Random(); index
				 * = random.nextInt(1 - 0 + 1) + 0; } else if (sizePackageList
				 * == 3) { Random random = new Random(); index =
				 * random.nextInt(2 - 1 + 1) + 1; } else if (sizePackageList ==
				 * 4) { Random random = new Random(); index = random.nextInt(3 -
				 * 2 + 1) + 2; }
				 */
				// Get the Package Data
				PackageData pData = currentAgent.lstPackage.get(index);

				// Send the Message, if the Package is not reserved
				if (!pData.isReserved()) {
					ACLMessage msgPackageSearch = new ACLMessage(
							MessageType.SEARCH_FOR_PACKAGE);
					AgentHelper.addReceiver(msgPackageSearch, myAgent,
							RampOrderAgent.NAME, currentAgent.conveyorID,
							currentAgent.szenarioID);

					if (Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName()
								+ " -> SEARCH_FOR_PACKAGE");

					try {
						msgPackageSearch.setContentObject(pData);
						send(msgPackageSearch);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}

			}
		}
	}
	
	/**
	 * Behaviour (Entry and Storage Behaviour) should cyclically check if the first package in the list is already reserved
	 * and then tell the Routingagent to start the Auction
	 * 
	 * @author Raschid
	 */
	private class InitializeAuctionStartBehaviour extends TickerBehaviour {

		public InitializeAuctionStartBehaviour(Agent a, long period) {
			super(a, period);

		}

		@Override
		protected void onTick() {
			PackageAgent currentAgent = (PackageAgent) myAgent;

			// Only if Jobs exists, there should be made a request
			int sizePackageList = currentAgent.lstPackage.size();
			if (sizePackageList >= 1) {
				
				// Get the Package Data
				PackageData pData = currentAgent.lstPackage.get(0);

				// Send the Message, if the Package is  reserved
				if (pData.isReserved()) {
					ACLMessage msgPackageAuctionStart = new ACLMessage(
							MessageType.INITIALIZE_START_AUCTION_BEHAVIOUR);
					msgPackageAuctionStart.addUserDefinedParameter("conveyorId",""+pData.getDestinationID());
					AgentHelper.addReceiver(msgPackageAuctionStart, myAgent,RampRoutingAgent.NAME, currentAgent.conveyorID,currentAgent.szenarioID);

					if (Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName()
								+ " -> INITIALIZE_START_AUCTION_BEHAVIOUR");

					try {
						msgPackageAuctionStart.setContentObject(pData);
						send(msgPackageAuctionStart);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}

			}
		}
	}

	/**
	 * Behaviour should receive the request from its Storage Orderagent and
	 * should search the list and look if the requested Package exists and then
	 * answer the Orderagent
	 * 
	 * @author Raschid
	 */
	private class AnswerIfPackageIsContainedBehaviour extends
			CyclicReceiverBehaviour {

		protected AnswerIfPackageIsContainedBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Orderagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData searchedPackage = null;

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- CHECK_IF_PACKAGE_IS_STORED");

			searchedPackage = (PackageData) msg.getContentObject();
			// get the Package id
			int id = searchedPackage.getPackageID();

			// Check if it is contained at the first position
			int size = currentAgent.lstPackage.size();
			if (size >= 1) {
				PackageData firstPositionPackage = currentAgent.lstPackage
						.get(0);
				if (id == firstPositionPackage.getPackageID()) {
					// Answer the Orderagent with Yes
					ACLMessage msgAnswer = new ACLMessage(MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
					msgAnswer.addUserDefinedParameter("answer_if_contained","Yes");
					msgAnswer.setContentObject(searchedPackage);
					msgAnswer.addReceiver(msg.getSender());
					if (Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " -> ANSWER_IF_PACKAGE_IS_CONTAINED: "+ msgAnswer.getUserDefinedParameter("answer_if_contained"));
					send(msgAnswer);

				} else {
					// Answer the Orderagent with No
					ACLMessage msgAnswer = new ACLMessage(MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
					msgAnswer.addUserDefinedParameter("answer_if_contained","No");
					msgAnswer.setContentObject(searchedPackage);
					msgAnswer.addReceiver(msg.getSender());
					if (Debugging.showInfoMessages)
						logger.log(Level.INFO,myAgent.getLocalName() + " -> ANSWER_IF_PACKAGE_IS_CONTAINED: "+ msgAnswer.getUserDefinedParameter("answer_if_contained"));
					send(msgAnswer);

				}
			} else {
				// Answer the Orderagent with No if there is no Package in the
				// list
				ACLMessage msgAnswer = new ACLMessage(
						MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
				msgAnswer.addUserDefinedParameter("answer_if_contained", "No");
				msgAnswer.setContentObject(searchedPackage);
				msgAnswer.addReceiver(msg.getSender());
				if (Debugging.showInfoMessages)
					logger.log(Level.INFO,myAgent.getLocalName() + " -> ANSWER_IF_PACKAGE_IS_CONTAINED: "+ msgAnswer.getUserDefinedParameter("answer_if_contained"));
				send(msgAnswer);
			}

		}

	}

	/**
	 * Behaviour should set a Package Reserved for an Exit
	 * 
	 * @author Raschid
	 */
	private class PackageReservationBehaviour extends CyclicReceiverBehaviour {

		protected PackageReservationBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Orderagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData receivedPackage = (PackageData) msg.getContentObject();

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- SET_PACKAGE_RESERVED");

			// Search the Package in the list and set it reserved
			for (int i = 0; i < currentAgent.lstPackage.size(); i++) {
				PackageData dummy = currentAgent.lstPackage.get(i);
				if (dummy.getPackageID() == receivedPackage.getPackageID()) {
					dummy.setReserved();
					break;

				}
			}
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ "PACKAGE_RESERVED");
        }

	}
	
	/**
	 * Behaviour should set a Package Reserved and set its destination
	 * 
	 * @author Raschid
	 */
	private class SetPackageDestinationBehaviour extends CyclicReceiverBehaviour {

		protected SetPackageDestinationBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Orderagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
			int destinationId= Integer.parseInt(msg.getUserDefinedParameter("conveyorId"));
			PackageData receivedPackage = (PackageData) msg.getContentObject();

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- SET_PACKAGE_RESERVED");

			// Search the Package in the list and set it reserved
			for (int i = 0; i < currentAgent.lstPackage.size(); i++) {
				PackageData dummy = currentAgent.lstPackage.get(i);
				if (dummy.getPackageID() == receivedPackage.getPackageID()) {
					dummy.setReserved();
					dummy.setDestinationID(destinationId);//The destination id is setted
					break;

				}
			}
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ "PACKAGE_RESERVED");
        }

	}

   /**
    * Checks the internal packageList, if there is an outgoing Job for the given Package, answer the RampOrderAgent
    * with PACKAGE_IS_NEEDED
    *
    * @author sijakubo
    */
   private class PackageNeedInformationBehaviour extends CyclicReceiverBehaviour {
      protected PackageNeedInformationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         EnquiredPackageData enquiredPackageData = (EnquiredPackageData) msg.getContentObject();

         for (PackageData requiredPackageData : lstPackage) {
            if (enquiredPackageData.getPackageData().getPackageID() == requiredPackageData.getPackageID()) {
               ACLMessage msgAnswer = new ACLMessage(MessageType.PACKAGE_IS_NEEDED);
               msgAnswer.setContentObject(enquiredPackageData);
               send(msgAnswer);
            }
         }
      }
   }

   /**
    * Behaviour to add a destinationId to a packageData
    *
    * @author sijakubo
    */
   private class AssignDestinationToPackageBehaviour extends CyclicReceiverBehaviour {

      protected AssignDestinationToPackageBehaviour(MessageTemplate mt) {
         super(mt);
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageDestinationData packageDestinationData = ((PackageDestinationData) msg.getContentObject());

         for (PackageData packageData : lstPackage) {
            if (packageData.getPackageID() == packageDestinationData.getPackageId()
                  && packageData.getDestinationID() != 0) {

               packageData.setDestinationID(packageDestinationData.getRampDestination());
               if (Debugging.showInfoMessages)
                  logger.log(Level.INFO, myAgent.getLocalName()
                        + ": destination from package " + packageData.getPackageID()
                        + " changed to destination " + packageData.getDestinationID());
            }
         }
      }
   }
   
   /**
	 * Behaviour should remove a Package in order to give it to the Volksbot
	 * 
	 * @author Raschid
	 */
	private class RemovePackageAndAnswerBehaviour extends CyclicReceiverBehaviour {

		protected RemovePackageAndAnswerBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Orderagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
			int packageId =Integer.parseInt(msg.getUserDefinedParameter("packageID"));
            PackageData packagee=null;
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- REMOVE_PACKAGE_AND_ANSWER");

			// Search the Package in the list and and remove it
			for (int i = 0; i < currentAgent.lstPackage.size(); i++) {
				packagee = currentAgent.lstPackage.get(i);
				if (packagee.getPackageID() == packageId) {
					packagee.setUnReserved();
					currentAgent.lstPackage.remove(i);
					break;
				}
			}
			
			//Answer the Plattformagent
			ACLMessage msgPackageRemoved = new ACLMessage(MessageType.PACKAGE_REMOVED);
			msgPackageRemoved.setContentObject(packagee);
			AgentHelper.addReceiver(msgPackageRemoved, currentAgent,RampPlattformAgent.NAME, currentAgent.conveyorID, currentAgent.szenarioID);
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ "->PACKAGE_REMOVED");
			send(msgPackageRemoved);
       }

	}
   
	/**
	 * Behaviour should charge a Package on the Bot
	 * 
	 * @author Raschid
	 */
	private class BotAddPackageBehaviour extends CyclicReceiverBehaviour {

		protected BotAddPackageBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from its Plattformagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
            PackageData packagee=(PackageData) msg.getContentObject();
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- BOT_ADD_PACKAGE");

			//Add the Package
			
			currentAgent.lstPackage.add(packagee);
							
       }

	}
	
	/**
	 * Behaviour should remove a Package from the Bot
	 * 
	 * @author Raschid
	 */
	private class BotRemovePackageBehaviour extends CyclicReceiverBehaviour {

		protected BotRemovePackageBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Bot
			PackageAgent currentAgent = (PackageAgent) myAgent;
 
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- BOT_REMOVE_PACKAGE");

			//Remove the Package and overgive it
			PackageData p=currentAgent.lstPackage.get(0);
			currentAgent.lstPackage.remove(0);
			
			ACLMessage packageRemoved = new ACLMessage(MessageType.BOT_REMOVED_PACKAGE);
			packageRemoved.setContentObject(p);
			packageRemoved.addReceiver(msg.getSender());
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_REMOVED_PACKAGE");
			
			send(packageRemoved);
			
				
			
			
       }

	}
	
	
	
	
	
   
}
