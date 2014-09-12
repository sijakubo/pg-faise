package uni.oldenburg.server.agent;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.apache.log4j.Logger;

import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.event.BotWorkloadChangedEvent;
import uni.oldenburg.shared.model.event.EventHelper;
import uni.oldenburg.shared.model.statistic.BotWorkloadDataModel;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Agent to track different Scenarios (e.g. How long does it take from Entrace - Exit of a package) throughout
 * the Simulation.
 *
 * @author sijakubo
 */
@SuppressWarnings("serial")
public class StatisticAgent extends Agent {
   public final static String AGENT_NAME = "StatisticAgent";

   public final static String PARAM_TIMESTAMP = "timestamp";
   public static final String PARAM_PACKAGE_ID = "packageId";
   public static final String PARAM_CONVEYOR_ID = "conveyorId";

   private Logger logger = Logger.getLogger(StatisticAgent.class);

   //Durchlaufzeit
   private Map<Integer, Long> packageEntrancetimestampsByPackageId = new HashMap<Integer, Long>();
   private List<Long> packageProcessingTimes = new ArrayList<Long>();

   //Auslastung Bots
   private Map<Integer, Long> lastContactTimestampByConveyorId = new HashMap<Integer, Long>();
   private long timeBotsWaited;
   private long timeBotsWorked;

   @Override
   /**
    * @author sijakubo
    */
   protected void setup() {
      addBehaviour(new PackageEnteredSimulationBehaviour(MessageType.PACKAGE_ENTERED_SIMULATION));
      addBehaviour(new PackageLeftSimulationBehaviour(MessageType.PACKAGE_LEFT_SIMULATION));

      addBehaviour(new BotCreatedBehaviour(MessageType.BOT_CREATED));
      addBehaviour(new BotStartedWorkingBehaviour(MessageType.BOT_STARTED_WORKING));
      addBehaviour(new BotStoppedWorkingBehaviour(MessageType.BOT_STOPPED_WORKING));

      Object[] args = getArguments();
      Szenario szenario = (Szenario) args[0];

      AgentHelper.getUniqueNickname(StatisticAgent.AGENT_NAME, -1, szenario.getId());
      AgentHelper.registerAgent(szenario.getId(), this, StatisticAgent.AGENT_NAME);
   }

   /**
    * @author sijakubo
    */
   private class PackageEnteredSimulationBehaviour extends CyclicReceiverBehaviour {
      protected PackageEnteredSimulationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      /**
       * Expected params:
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_PACKAGE_ID}
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_TIMESTAMP}
       */
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         packageEntrancetimestampsByPackageId.put(
               Integer.valueOf(msg.getUserDefinedParameter(PARAM_PACKAGE_ID)),
               Long.valueOf(msg.getUserDefinedParameter(PARAM_TIMESTAMP))
         );
      }
   }

   /**
    * @author sijakubo
    */
   private class PackageLeftSimulationBehaviour extends CyclicReceiverBehaviour {
      protected PackageLeftSimulationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      /**
       * Expected params:
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_PACKAGE_ID}
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_TIMESTAMP}
       */
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         Long packageEntranceTime =
               packageEntrancetimestampsByPackageId.get(Integer.valueOf(msg.getUserDefinedParameter(PARAM_PACKAGE_ID)));
         Long packageLeftTime = Long.valueOf(msg.getUserDefinedParameter(PARAM_TIMESTAMP));
         packageProcessingTimes.add(packageLeftTime - packageEntranceTime);
      }
   }

   /**
    * @author sijakubo
    */
   private class BotCreatedBehaviour extends CyclicReceiverBehaviour {
      protected BotCreatedBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      /**
       * Expected params:
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_CONVEYOR_ID}
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_TIMESTAMP}
       */
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         lastContactTimestampByConveyorId.put(
               Integer.valueOf(msg.getUserDefinedParameter(PARAM_CONVEYOR_ID)),
               Long.valueOf(msg.getUserDefinedParameter(PARAM_TIMESTAMP))
         );
      }
   }

   /**
    * @author sijakubo
    */
   private class BotStartedWorkingBehaviour extends CyclicReceiverBehaviour {
      protected BotStartedWorkingBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      /**
       * Expected params:
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_CONVEYOR_ID}
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_TIMESTAMP}
       */
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         Integer conveyorId = Integer.valueOf(msg.getUserDefinedParameter(PARAM_CONVEYOR_ID));
         Long currentTimestamp = Long.valueOf(msg.getUserDefinedParameter(PARAM_TIMESTAMP));

         Long lastContactTimestamp = lastContactTimestampByConveyorId.remove(conveyorId);
         timeBotsWaited = timeBotsWaited + (currentTimestamp - lastContactTimestamp);
         lastContactTimestampByConveyorId.put(conveyorId, currentTimestamp);

         EventHelper.addEvent(new BotWorkloadChangedEvent(
               new BotWorkloadDataModel(1, "Wartend", timeBotsWaited),
               new BotWorkloadDataModel(2, "Arbeitend", timeBotsWorked)));

         logger.info("Bot started working " + timeBotsWorked + " / " + timeBotsWaited);
      }
   }

   /**
    * @author sijakubo
    */
   private class BotStoppedWorkingBehaviour extends CyclicReceiverBehaviour {
      protected BotStoppedWorkingBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      /**
       * Expected params:
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_CONVEYOR_ID}
       *    {@link uni.oldenburg.server.agent.StatisticAgent#PARAM_TIMESTAMP}
       */
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         Integer conveyorId = Integer.valueOf(msg.getUserDefinedParameter(PARAM_CONVEYOR_ID));
         Long currentTimestamp = Long.valueOf(msg.getUserDefinedParameter(PARAM_TIMESTAMP));

         Long lastContactTimestamp = lastContactTimestampByConveyorId.remove(conveyorId);
         timeBotsWorked = timeBotsWorked + (currentTimestamp - lastContactTimestamp);
         lastContactTimestampByConveyorId.put(conveyorId, currentTimestamp);

         EventHelper.addEvent(new BotWorkloadChangedEvent(
               new BotWorkloadDataModel(1, "Wartend", timeBotsWaited),
               new BotWorkloadDataModel(2, "Arbeitend", timeBotsWorked)));

         logger.info("Bot stopped working " + timeBotsWorked + " / " + timeBotsWaited);
      }
   }
}
