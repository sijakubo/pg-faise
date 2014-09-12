package uni.oldenburg.shared.model.statistic;

import java.io.Serializable;

/**
 * @author sijakubo
 */
@SuppressWarnings("serial")
public class PackageProcessingTimeDataModel implements Serializable {
   private int id;
   private String state;
   private int timeSpan;

   public PackageProcessingTimeDataModel() {
   }

   public PackageProcessingTimeDataModel(int id, String state, int timeSpan) {
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

   public int getTimeSpan() {
      return timeSpan;
   }

   public void setTimeSpan(Integer timeSpan) {
      this.timeSpan = timeSpan;
   }
}
