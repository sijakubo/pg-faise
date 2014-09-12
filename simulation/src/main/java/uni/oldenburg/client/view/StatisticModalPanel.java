package uni.oldenburg.client.view;

import com.google.gwt.core.client.GWT;
import com.google.gwt.user.client.ui.IsWidget;
import com.google.gwt.user.client.ui.Widget;
import com.sencha.gxt.chart.client.chart.Chart;
import com.sencha.gxt.chart.client.chart.Legend;
import com.sencha.gxt.chart.client.chart.series.PieSeries;
import com.sencha.gxt.chart.client.chart.series.Series;
import com.sencha.gxt.chart.client.chart.series.SeriesLabelConfig;
import com.sencha.gxt.chart.client.draw.Gradient;
import com.sencha.gxt.chart.client.draw.RGB;
import com.sencha.gxt.chart.client.draw.Stop;
import com.sencha.gxt.core.client.ValueProvider;
import com.sencha.gxt.data.shared.*;
import com.sencha.gxt.widget.core.client.Window;
import com.sencha.gxt.widget.core.client.button.TextButton;
import com.sencha.gxt.widget.core.client.event.HideEvent;
import com.sencha.gxt.widget.core.client.event.SelectEvent;

import uni.oldenburg.shared.model.statistic.BotWorkloadDataModel;


/**
 * @author sijakubo
 */
public class StatisticModalPanel implements IsWidget {
   private static final BotWorkloadDataModelProperties botWorkloadDataModelProperties = GWT.create(BotWorkloadDataModelProperties.class);
   private Chart<BotWorkloadDataModel> botWorkloadChart;

   public interface BotWorkloadDataModelProperties extends PropertyAccess<BotWorkloadDataModel> {
      ModelKeyProvider<BotWorkloadDataModel> id();

      ValueProvider<BotWorkloadDataModel, String> state();

      ValueProvider<BotWorkloadDataModel, Long> timeSpan();
   }

   private final Window window = new Window();

   public StatisticModalPanel() {
      initWindow();
      botWorkloadChart = createBotWorkloadChart();
      window.add(botWorkloadChart);
   }

   @SuppressWarnings("unused")
private Chart<BotWorkloadDataModel> createPackageProcessingTimeChart() {
      return null;
   }

   private Chart<BotWorkloadDataModel> createBotWorkloadChart() {
      Chart<BotWorkloadDataModel> chart = new Chart<BotWorkloadDataModel>();
      chart.setStore(new ListStore<BotWorkloadDataModel>(botWorkloadDataModelProperties.id()));

      //Initial Data
      chart.getStore().add(new BotWorkloadDataModel(1, "Wartend", 50L));
      chart.getStore().add(new BotWorkloadDataModel(2, "Arbeitend", 50L));

      // Set the chart legend
      Legend<BotWorkloadDataModel> legend = new Legend<BotWorkloadDataModel>();
      legend.setPosition(Chart.Position.RIGHT);
      legend.setItemHighlighting(true);
      legend.setItemHiding(true);
      chart.setLegend(legend);

      // Setup series slice colors
      Gradient slice1 = new Gradient(45);
      slice1.addStop(new Stop(0, new RGB(148, 174, 10)));
      slice1.addStop(new Stop(100, new RGB(107, 126, 7)));
      chart.addGradient(slice1);
      Gradient slice2 = new Gradient(45);
      slice2.addStop(new Stop(0, new RGB(17, 95, 166)));
      slice2.addStop(new Stop(100, new RGB(12, 69, 120)));
      chart.addGradient(slice2);

      // Setup the chart series
      PieSeries<BotWorkloadDataModel> series = new PieSeries<BotWorkloadDataModel>();
      series.setAngleField(botWorkloadDataModelProperties.timeSpan());
      series.setDonut(0);
      series.addColor(slice1);
      series.addColor(slice2);

      // Setup the label config
      SeriesLabelConfig<BotWorkloadDataModel> labelConfig = new SeriesLabelConfig<BotWorkloadDataModel>();
      labelConfig.setLabelPosition(Series.LabelPosition.START);
      labelConfig.setValueProvider(botWorkloadDataModelProperties.state(), new StringLabelProvider<String>());
      series.setLabelConfig(labelConfig);

      // Setup the legend label config
      series.setLegendValueProvider(botWorkloadDataModelProperties.state(), new LabelProvider<String>() {
         public String getLabel(String item) {
            return item.substring(0, 3);
         }
      });
      chart.addSeries(series);
      return chart;
   }

   private void initWindow() {
      window.setVisible(false);
      window.setPixelSize(500, 300);
      window.setModal(false);
      window.setBlinkModal(true);
      window.setHeadingText("Statistiken");
      window.addHideHandler(new HideEvent.HideHandler() {
         public void onHide(HideEvent event) {
            TextButton open = ((Window) event.getSource()).getData("open");
            open.focus();
         }
      });

      TextButton closeButton = new TextButton("Close");
      closeButton.addSelectHandler(new SelectEvent.SelectHandler() {
         public void onSelect(SelectEvent event) {
            window.hide();
         }
      });
      window.addButton(closeButton);
      window.setFocusWidget(window.getButtonBar().getWidget(0));
   }

   public void show() {
      this.window.show();
   }

   public Widget asWidget() {
      return window;
   }

   public void newBotWorkloadDataReceived(BotWorkloadDataModel dataModelBotWaited,
                                          BotWorkloadDataModel dataModelBotWorked) {

      botWorkloadChart.getStore().clear();
      botWorkloadChart.getStore().add(dataModelBotWaited);
      botWorkloadChart.getStore().add(dataModelBotWorked);
      botWorkloadChart.redrawChart();
   }
}
