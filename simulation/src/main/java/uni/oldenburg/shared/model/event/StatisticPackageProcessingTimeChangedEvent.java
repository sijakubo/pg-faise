package uni.oldenburg.shared.model.event;

import de.novanic.eventservice.client.event.Event;
import uni.oldenburg.shared.model.statistic.PackageProcessingTimeDataModel;

@SuppressWarnings("serial")
/**
 * @author sijakubo
 */
public class StatisticPackageProcessingTimeChangedEvent implements Event {
   private PackageProcessingTimeDataModel packageProcessingTimeDataModel;

   public StatisticPackageProcessingTimeChangedEvent() {
   }

   public StatisticPackageProcessingTimeChangedEvent(PackageProcessingTimeDataModel packageProcessingTimeDataModel) {
      this.packageProcessingTimeDataModel = packageProcessingTimeDataModel;
   }

   public PackageProcessingTimeDataModel getPackageProcessingTimeDataModel() {
      return packageProcessingTimeDataModel;
   }

   public void setPackageProcessingTimeDataModel(PackageProcessingTimeDataModel packageProcessingTimeDataModel) {
      this.packageProcessingTimeDataModel = packageProcessingTimeDataModel;
   }
}