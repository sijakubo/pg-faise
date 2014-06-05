package uni.oldenburg.client.view;

import com.google.gwt.core.client.GWT;
import com.google.gwt.resources.client.ClientBundle;
import com.google.gwt.resources.client.DataResource;
import com.google.gwt.resources.client.ImageResource;
import com.google.gwt.user.client.ui.Image;
import com.kiouri.sliderbar.client.view.SliderBarHorizontal;
import com.kiouri.sliderbar.client.view.SliderBarVertical;



public class SliderBarSimpleHorizontal extends SliderBarHorizontal {
    
    ImagesSliderBarSimpleHorizontal images = GWT.create(ImagesSliderBarSimpleHorizontal.class);

    public SliderBarSimpleHorizontal( String width, boolean showRows) {         
            if (showRows){
                    setLessWidget(new Image(images.less()) );
                    setScaleWidget(new Image(images.scalev().getUrl()), 10);
                    setMoreWidget(new Image(images.more()));
            } else {
                setScaleWidget(new Image(images.scalev().getUrl()), 10);
            }
            setDragWidget(new Image(images.drag()));
            this.setWidth(width);
            this.setMaxValue(3);             
    }
    

    interface ImagesSliderBarSimpleHorizontal extends ClientBundle{
            
            @Source("kdehdrag.png")
            ImageResource drag();

            @Source("kdehless.png")
            ImageResource less();

            @Source("kdehmore.png")
            ImageResource more();

            @Source("kdehscale.png")
            DataResource scalev();                          
    }
    
}