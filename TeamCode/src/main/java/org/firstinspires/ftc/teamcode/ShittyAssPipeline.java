package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShittyAssPipeline extends OpenCvPipeline {

    Mat YCbCr = new Mat();
    Mat leftFrame;
    Mat centerFrame;
    Mat rightFrame;
    double leftAvgf;
    double centerAvgf;
    double rightAvgf;
    boolean isL = false;
    boolean isC = false;
    boolean isR = false;

    Mat output = new Mat();
    Scalar rectColor = new Scalar(255, 0,0);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(113, 200, 100, 100);
        Rect centerRect = new Rect(213, 200, 100, 100);
        Rect rightRect = new Rect(313, 200, 100, 100);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect,rectColor, 2);
        Imgproc.rectangle(output, centerRect,rectColor, 2);
        Imgproc.rectangle(output, rightRect,rectColor, 2);

        leftFrame = YCbCr.submat(leftRect);
        centerFrame = YCbCr.submat(centerRect);
        rightFrame = YCbCr.submat(rightRect);


        Core.extractChannel(leftFrame, leftFrame,2);
        Core.extractChannel(centerFrame, centerFrame,2);
        Core.extractChannel(rightFrame, rightFrame,2);

        Scalar leftAvg = Core.mean(leftFrame);
        Scalar centerAvg = Core.mean(centerFrame);
        Scalar rightAvg = Core.mean(rightFrame);

        leftAvgf = leftAvg.val[0];
        centerAvgf = centerAvg.val[0];
        rightAvgf = rightAvg.val[0];


        if(leftAvgf > centerAvgf){
            if(leftAvgf > rightAvgf){
                isL = true;
            }else{
                isR = true;
            }
        }else {
            if(centerAvgf > rightAvgf){
                isC = true;
            }else{
                isR = true;
            }
        }



        return output;
    }
}
