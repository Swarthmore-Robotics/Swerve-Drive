package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class Vision {

    // static singleton vision class instance
    private static Vision instance = null;
    // Public Variables
    public int imgWidth;
    public int imgHeight;
    public Rect biggestRed;
    public Rect biggestYellow;
    public Rect biggestGreen;
    public Rect[] biggestColors = {biggestRed, biggestYellow, biggestGreen};
    // Private Variables
    private Thread visionThread;
    private RectAreaComparator rectSorter;
    private Size gb = new Size(7, 7); // gaussian blur
    // Private Final Variables
    private final Rect EMPTY_RECT = new Rect(0, 0, 0, 0);
    private Mat BLACK_IMAGE;
    private final int MAX_OBJECT_DETECT = 5;
    private final boolean VERBOSE = false;
    // colors
    private static final Scalar RED_COLOR = new Scalar(0, 0, 255);
    private static final Scalar YELLOW_COLOR = new Scalar(0, 255, 255);
    private static final Scalar GREEN_COLOR = new Scalar(0, 255, 0);
    private static final Scalar BLUE_COLOR = new Scalar(255, 0, 0);
    private static final Scalar PINK_COLOR = new Scalar(255, 0, 255);

    // BGR thresholding values
    // TODO: put into constants file
    private Scalar redLower = new Scalar(0, 0, 55);
    private Scalar redUpper = new Scalar(80, 35, 255);

    private Scalar yellowLower = new Scalar(0, 150, 175);
    private Scalar yellowUpper = new Scalar(140, 200, 230);

    private Scalar greenLower = new Scalar(0, 180, 0);
    private Scalar greenUpper = new Scalar(170, 255, 200);

    /*
     * Constructors
     */
    private Vision(int imageWidth, int imageHeight){
        this.imgWidth = imageWidth;
        this.imgHeight = imageHeight;
        this.rectSorter = new RectAreaComparator();
        this.init();
    }
    private Vision(){
        this(320, 240);
    } 

    /*
     * Accessor to Vision
     * - uses singleton design pattern
     */
    public static Vision getInstance(){
        if(instance == null){
            instance = new Vision();
        }
        
        return instance;
    }
    public static Vision getInstance(int a, int b){
        if(instance == null){
            instance = new Vision(a, b);
        }
        
        return instance;
    }

    /*
     * Rect Comparator Class
     * - compares Rect from smallest to largest area
     */
    class RectAreaComparator implements Comparator<Rect>{

        @Override
        public int compare(Rect r1, Rect r2) {
            return (int) (r1.area() - r2.area());
        }
        
    }

    /*
     * Color Detector Class
     * - detects specified color
     * - assumes gaussian blur has been applies
     * 
     * 1. thresholds on color
     * 2. finds contours
     * 3. finds bounding boxes on contours
     * 4. updates to list of detected rectangles & biggest rectangle color
     */

    class ColorDetector{
        private Scalar color;
        private Scalar upperLimit;
        private Scalar lowerLimit;
        private Mat mask;
        private Mat tempImage;
        private List<MatOfPoint> contours;
        private Mat hierarchy;
        private Rect biggestRect;
        private List<Rect> detectedRects;
        private Rect br;
        private Rect temp;

        public ColorDetector(Scalar c, Scalar ll, Scalar ul){
            color = c;
            upperLimit = ul;
            lowerLimit = ll;

            mask = new Mat();
            hierarchy = new Mat();
            contours = new ArrayList<MatOfPoint>();
            detectedRects = new ArrayList<Rect>();
        }

        private void reset(){
            BLACK_IMAGE.copyTo(mask);
            BLACK_IMAGE.copyTo(hierarchy);
            contours.clear();
        }

        private Rect findRects(Mat image){

            reset();

            Core.inRange(image, this.lowerLimit, this.upperLimit, this.mask);

            Imgproc.findContours(this.mask, this.contours, this.hierarchy, Imgproc.RETR_EXTERNAL,
                            Imgproc.CHAIN_APPROX_SIMPLE);

            // BLACK_IMAGE.copyTo(tempImage);

            if (this.contours.size() == 0) {
                temp = EMPTY_RECT;
            } else {
                this.temp = EMPTY_RECT;
                this.detectedRects.clear();
                for (int i = 0; i < Math.min(this.contours.size(), MAX_OBJECT_DETECT); i++) {
                    // Imgproc.drawContours(sourceMat, contours, i, redColor, 2, Imgproc.LINE_8,
                    // hierarchy, 0, new Point());
                    br = Imgproc.boundingRect(this.contours.get(i));
                    this.detectedRects.add(br);
                    if (br.area() > temp.area()) {
                        this.temp = br;
                    }
                    if (VERBOSE) {
                        Imgproc.rectangle(image, br.tl(), br.br(), color, 1);
                    }
                }

                // sort detected rectangles from biggest to smallest area
                Collections.sort(this.detectedRects, rectSorter);
                Collections.reverse(this.detectedRects);
            }
            this.biggestRect = this.temp;

            return this.biggestRect;
        }
    }

    /*
     * Calculate center of rectangle
     */
    public Point rectCenter(Rect r) {
        return new Point(r.tl().x + (r.width / 2.0), r.tl().y + (r.height / 2.0));
    }

    /*
     * Convert Point to array [x, y]
     * 
     */
    public double[] point2array(Point p){
        double[] arr = {p.x, p.y};

        return arr;
    }

    /*
     * Initializes Computer Vision
     */
    private void init() {

        visionThread = new Thread(() -> {
            // add USB camera, create server for SmartDashboard
            UsbCamera usbCamera = CameraServer.startAutomaticCapture("Main Camera", 0);
            usbCamera.setResolution(imgWidth, imgHeight);

            CvSink cvSink = CameraServer.getVideo(); // grab images from camera
            CvSource outputStream = CameraServer.putVideo("Processed Image", imgWidth, imgHeight);
            

            // Init variables
            Mat sourceMat = new Mat();
            Mat outputMat = new Mat();
            BLACK_IMAGE = Mat.zeros(imgHeight, imgWidth, 16);
            BLACK_IMAGE.copyTo(sourceMat);
            BLACK_IMAGE.copyTo(outputMat);

            ColorDetector redDetector = new ColorDetector(RED_COLOR, redLower, redUpper);
            ColorDetector yellowDetector = new ColorDetector(YELLOW_COLOR, yellowLower, yellowUpper);
            ColorDetector greenDetector = new ColorDetector(GREEN_COLOR, greenLower, greenUpper);

            List<ColorDetector> detectors = new ArrayList<ColorDetector>();
            detectors.add(redDetector);
            detectors.add(yellowDetector);
            detectors.add(greenDetector);

            int numDetectors = detectors.size();

            while (true) { /// TODO: change condition later
                long startTime = System.currentTimeMillis();

                if (cvSink.grabFrame(sourceMat) != 0) {

                    // gaussian blur
                    Imgproc.GaussianBlur(sourceMat, sourceMat, gb, 0, 0);

                    BLACK_IMAGE.copyTo(outputMat);

                    // color detection
                    for(int i = 0; i < numDetectors; i++){
                    // for(int i = 0; i < 1; i++){
                        ColorDetector cd = detectors.get(i);
                        biggestColors[i] = cd.findRects(sourceMat);
                        // System.out.println("found red");
                        outputMat.setTo(cd.color, cd.mask);
                        if (!VERBOSE) {
                            Imgproc.rectangle(outputMat, biggestColors[i].tl(), biggestColors[i].br(), cd.color, 1);
                            Imgproc.circle(outputMat, rectCenter(biggestColors[i]), 3, BLUE_COLOR, -1);
                        }
                    }

                    biggestRed = biggestColors[0];
                    biggestYellow = biggestColors[1];
                    biggestGreen = biggestColors[2];

                    // output results
                    outputStream.putFrame(outputMat); // put processed image to smartdashboard
                    long endTime = System.currentTimeMillis();

                    // System.out.println("Total execution time: " + (endTime - startTime));

                    SmartDashboard.putString("Biggest Red Center", rectCenter(biggestRed).toString());
                    SmartDashboard.putString("Biggest Yellow Center", rectCenter(biggestYellow).toString());
                    SmartDashboard.putString("Biggest Green Center", rectCenter(biggestGreen).toString());
                }

            }

        });

        visionThread.setPriority(10); // highest priority
    }

    public void startVision() {
        visionThread.start(); // start vision thread
    }

}
