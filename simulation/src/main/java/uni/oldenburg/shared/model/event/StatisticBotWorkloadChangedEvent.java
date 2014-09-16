package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;
import uni.oldenburg.shared.model.statistic.BotWorkloadDataModel;

@SuppressWarnings("serial")
/**
 * @author sijakubo
 */
public class StatisticBotWorkloadChangedEvent implements Event {
   private BotWorkloadDataModel botWaitedDataModel;
   private BotWorkloadDataModel botWorkedDataModel;

   public StatisticBotWorkloadChangedEvent() {
   }

   public StatisticBotWorkloadChangedEvent(BotWorkloadDataModel botWaitedDataModel, BotWorkloadDataModel botWorkedDataModel) {
      this.botWaitedDataModel = botWaitedDataModel;
      this.botWorkedDataModel = botWorkedDataModel;
   }

   public BotWorkloadDataModel getBotWaitedDataModel() {
      return botWaitedDataModel;
   }

   public void setBotWaitedDataModel(BotWorkloadDataModel botWaitedDataModel) {
      this.botWaitedDataModel = botWaitedDataModel;
   }

   public BotWorkloadDataModel getBotWorkedDataModel() {
      return botWorkedDataModel;
   }

   public void setBotWorkedDataModel(BotWorkloadDataModel botWorkedDataModel) {
      this.botWorkedDataModel = botWorkedDataModel;
   }
}
