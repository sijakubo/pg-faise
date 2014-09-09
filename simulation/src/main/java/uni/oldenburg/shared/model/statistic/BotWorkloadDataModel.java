package uni.oldenburg.shared.model.statistic;

import java.io.Serializable;

/**
 * @author sijakubo
 */
public class BotWorkloadDataModel implements Serializable {
   private int id;
   private String state;
   private long timeSpan;

   public BotWorkloadDataModel() {
   }

   public BotWorkloadDataModel(int id, String state, long timeSpan) {
      this.id = id;
      this.state = state;
      this.timeSpan = timeSpan;
   }

   public int getId() {
      return id;
   }

   public void setId(int id) {
      this.id = id;
   }

   public String getState() {
      return state;
   }

   public void setState(String state) {
      this.state = state;
   }

   public long getTimeSpan() {
      return timeSpan;
   }

   public void setTimeSpan(long timeSpan) {
      this.timeSpan = timeSpan;
   }
}
