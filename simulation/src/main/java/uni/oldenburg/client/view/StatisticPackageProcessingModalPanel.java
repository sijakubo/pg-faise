package uni.oldenburg.client.view;

import com.google.gwt.user.client.ui.IsWidget;
import com.google.gwt.user.client.ui.Widget;
import com.sencha.gxt.chart.client.chart.Chart;
import com.sencha.gxt.chart.client.chart.Legend;
import com.sencha.gxt.chart.client.chart.axis.NumericAxis;
import com.sencha.gxt.chart.client.chart.series.LineSeries;
import com.sencha.gxt.chart.client.chart.series.Primitives;
import com.sencha.gxt.chart.client.chart.series.SeriesHighlighter;
import com.sencha.gxt.chart.client.draw.Color;
import com.sencha.gxt.chart.client.draw.DrawFx;
import com.sencha.gxt.chart.client.draw.RGB;
import com.sencha.gxt.chart.client.draw.path.PathSprite;
import com.sencha.gxt.chart.client.draw.sprite.Sprite;
import com.sencha.gxt.chart.client.draw.sprite.TextSprite;
import com.sencha.gxt.core.client.ValueProvider;
import com.sencha.gxt.data.shared.ListStore;
import com.sencha.gxt.data.shared.ModelKeyProvider;
import com.sencha.gxt.widget.core.client.Window;
import com.sencha.gxt.widget.core.client.button.TextButton;
import com.sencha.gxt.widget.core.client.event.HideEvent;
import com.sencha.gxt.widget.core.client.event.SelectEvent;
import uni.oldenburg.shared.model.statistic.PackageProcessingTimeDataModel;


/**
 * @author sijakubo
 */
public class StatisticPackageProcessingModalPanel implements IsWidget {
   private final Window window = new Window();
   private Chart<PackageProcessingTimeDataModel> packageProcessingTimeChart;

   public StatisticPackageProcessingModalPanel() {
      initWindow();

      packageProcessingTimeChart = createPackageProcessingTimeChart();
      window.add(packageProcessingTimeChart);
   }

   private Chart<PackageProcessingTimeDataModel> createPackageProcessingTimeChart() {
      final ListStore<PackageProcessingTimeDataModel> store = new ListStore<PackageProcessingTimeDataModel>(
            new ModelKeyProvider<PackageProcessingTimeDataModel>() {

               public String getKey(PackageProcessingTimeDataModel item) {
                  return "key" + item.getNewProcessingTime();
               }
            }
      );

      final Chart<PackageProcessingTimeDataModel> chart = new Chart<PackageProcessingTimeDataModel>();
      chart.setStore(store);
      chart.setShadowChart(false);
      chart.setAnimated(true);

      LineSeries<PackageProcessingTimeDataModel> series = new LineSeries<PackageProcessingTimeDataModel>();
      series.setYAxisPosition(Chart.Position.LEFT);
      series.setStroke(RGB.RED);
      series.setStrokeWidth(3);
      series.setShowMarkers(true);
      Sprite marker = Primitives.circle(0, 0, 6);
      marker.setFill(RGB.RED);
      series.setMarkerConfig(marker);

      series.setYField(new ValueProvider<PackageProcessingTimeDataModel, Number>() {
         public Number getValue(PackageProcessingTimeDataModel object) {
            return object.getNewProcessingTime();
         }

         public void setValue(PackageProcessingTimeDataModel object, Number value) {
            object.setNewProcessingTime(value.longValue());
         }

         public String getPath() {
            return "Paket-Durchlaufzeit";
         }
      });

      series.setLineHighlighter(new SeriesHighlighter() {
         public void highlight(Sprite sprite) {
            DrawFx.createStrokeWidthAnimator(sprite, 6).run(250);
         }

         public void unHighlight(Sprite sprite) {
            DrawFx.createStrokeWidthAnimator(sprite, 3).run(250);
         }
      });

      chart.addSeries(series);
      NumericAxis<PackageProcessingTimeDataModel> axis = new NumericAxis<PackageProcessingTimeDataModel>();

      axis.addField(series.getYField());
      axis.setPosition(Chart.Position.LEFT);
      TextSprite title = new TextSprite("Zeit in Sekunden");
      title.setFontSize(14);
      axis.setTitleConfig(title);
      axis.setMinorTickSteps(1);
      axis.setDisplayGrid(true);
      PathSprite odd = new PathSprite();
      odd.setOpacity(1);
      odd.setFill(new Color("#ddd"));
      odd.setStroke(new Color("#bbb"));
      odd.setStrokeWidth(0.5);
      axis.setGridOddConfig(odd);
      chart.addAxis(axis);

      Legend<PackageProcessingTimeDataModel> legend = new Legend<PackageProcessingTimeDataModel>();
      legend.setItemHighlighting(true);
      legend.setItemHiding(true);
      legend.getBorderConfig().setStrokeWidth(0);
      chart.setLegend(legend);
      return chart;
   }


   private void initWindow() {
      window.setVisible(false);
      window.setPixelSize(500, 300);
      window.setModal(false);
      window.setBlinkModal(true);
      window.setHeadingText("Paketdurchlaufzeit");
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
      packageProcessingTimeChart.redrawChart();
   }

   public Widget asWidget() {
      return window;
   }

   public void newPackageProcessingTimeDataReceived(PackageProcessingTimeDataModel packageProcessingTimesDataModel) {
      packageProcessingTimeChart.getStore().add(packageProcessingTimesDataModel);
      packageProcessingTimeChart.redrawChart();
   }
}
