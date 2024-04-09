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
    private static Constants C = new Constants();
    // Public Variables
    public int imgWidth;
    public int imgHeight;
    public Rect biggestRed;
    public Rect biggestYellow;
    public Rect biggestGreen;
    public Rect[] biggestColors = { biggestRed, biggestYellow, biggestGreen };
    // Private Variables
    private Thread visionThread;
    private RectAreaComparator rectSorter;
    // private Size gb = new Size(7, 7); // gaussian blur
    // Private Final Variables
    // private final Rect EMPTY_RECT = new Rect(0, 0, 0, 0);
    private Mat BLACK_IMAGE;

    /*
     * Constructors
     */
    private Vision(int imageWidth, int imageHeight) {
        this.imgWidth = imageWidth;
        this.imgHeight = imageHeight;
        this.rectSorter = new RectAreaComparator();
        this.init();
    }

    private Vision() {
        this(320, 240);
    }

    /*
     * Accessor to Vision
     * - uses singleton design pattern
     */
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    public static Vision getInstance(int a, int b) {
        if (instance == null) {
            instance = new Vision(a, b);
        }

        return instance;
    }

    /*
     * Rect Comparator Class
     * - compares Rect from smallest to largest area
     */
    class RectAreaComparator implements Comparator<Rect> {

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

    class ColorDetector {
        private Scalar color;
        private Scalar upperLimit;
        private Scalar lowerLimit;
        private List<MatOfPoint> contours;
        private Mat hierarchy;
        private Mat tempImage;
        private Rect biggestRect;
        private List<Rect> detectedRects;
        private Rect br;
        private Rect temp;

        public Mat mask;
        public Mat contourImage;
        public Mat contourBoundingBoxes;
        public Mat boundingBoxes;

        public ColorDetector(Scalar c, Scalar ll, Scalar ul) {
            color = c;
            upperLimit = ul;
            lowerLimit = ll;

            mask = new Mat();
            hierarchy = new Mat();
            boundingBoxes = new Mat();
            tempImage = new Mat();
            contourImage = new Mat();
            contourBoundingBoxes = new Mat();
            contours = new ArrayList<MatOfPoint>();
            detectedRects = new ArrayList<Rect>();
        }

        private void reset() {
            BLACK_IMAGE.copyTo(mask);
            BLACK_IMAGE.copyTo(hierarchy);
            BLACK_IMAGE.copyTo(boundingBoxes);
            BLACK_IMAGE.copyTo(contourBoundingBoxes);
            BLACK_IMAGE.copyTo(contourImage);
            contours.clear();
        }

        private Rect findRects(Mat image) {

            reset();

            Core.inRange(image, this.lowerLimit, this.upperLimit, this.mask);


            Imgproc.findContours(this.mask, this.contours, this.hierarchy, Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE);

            BLACK_IMAGE.copyTo(contourImage);
            BLACK_IMAGE.copyTo(boundingBoxes);
            BLACK_IMAGE.copyTo(contourBoundingBoxes);

            if (this.contours.size() == 0) {
                temp = C.EMPTY_RECT;
            } else {
                this.temp = C.EMPTY_RECT;
                this.detectedRects.clear();
                for (int i = 0; i < Math.min(this.contours.size(), C.MAX_OBJECT_DETECT); i++) {
                    Imgproc.drawContours(contourImage, this.contours, i, C.WHITE_COLOR, 1, Imgproc.LINE_8, this. hierarchy, 0);
                    Imgproc.drawContours(contourBoundingBoxes, this.contours, i, C.WHITE_COLOR, 1, Imgproc.LINE_8, this. hierarchy, 0);
                    br = Imgproc.boundingRect(this.contours.get(i));
                    this.detectedRects.add(br);
                    if (br.area() > temp.area()) {
                        this.temp = br;
                    }
                    if (C.VERBOSE) {
                        Imgproc.rectangle(image, br.tl(), br.br(), this.color, 1);
                    }else{
                        Imgproc.rectangle(boundingBoxes, br.tl(), br.br(), this.color, 1);
                        Imgproc.rectangle(contourBoundingBoxes, br.tl(), br.br(), this.color, 1);
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
    public double[] point2array(Point p) {
        double[] arr = { p.x, p.y };

        return arr;
    }

    /*
     * Initializes Computer Vision
     */
    private void init() {

        // add USB camera, create server for SmartDashboard
        // http://roborio-6593-frc.local:1181/
        UsbCamera usbCamera = CameraServer.startAutomaticCapture("Main Camera", 0);
        usbCamera.setResolution(imgWidth, imgHeight);
        usbCamera.setExposureManual(7);
        // usbCamera.setExposureAuto();
        usbCamera.setWhiteBalanceManual(3300);
        usbCamera.setBrightness(60); // default 50

        CvSink cvSink = CameraServer.getVideo(); // grab images from camera
        CvSource outputStream = CameraServer.putVideo("Processed Image", imgWidth, imgHeight);
        CvSource outputStreamGB = CameraServer.putVideo("Gaussian Blur", imgWidth, imgHeight);
        CvSource outputStreamColorThresh = CameraServer.putVideo("Color Threshold", imgWidth, imgHeight);
        CvSource outputStreamContours = CameraServer.putVideo("Contours", imgWidth, imgHeight);
        CvSource outputStreamContoursBB = CameraServer.putVideo("Contours And Bounding Boxes", imgWidth, imgHeight);
        CvSource outputStreamBB = CameraServer.putVideo("Bounding Boxes", imgWidth, imgHeight);
        CvSource outputStreamArea = CameraServer.putVideo("Biggest Area Center", imgWidth, imgHeight);
        CvSource outputStreamFinal = CameraServer.putVideo("Final", imgWidth, imgHeight);

        // Init variables
        Mat sourceMat = new Mat();
        Mat outputMat =new Mat();
        Mat tempMat = new Mat();
        Mat redAreaCenter = new Mat();
        BLACK_IMAGE = Mat.zeros(imgHeight, imgWidth, 16);
        BLACK_IMAGE.copyTo(sourceMat);
        BLACK_IMAGE.copyTo(outputMat);
        BLACK_IMAGE.copyTo(tempMat);
        Point centerTop = new Point(160,0);
        Point centerBottom = new Point(160,240);
        Point centerTopLeft = new Point(160 - C.centerThresh, 0);
        Point centerBottomRight = new Point(160 + C.centerThresh, 240);

        ColorDetector redDetector = new ColorDetector(C.RED_COLOR, C.redLower, C.redUpper);
        ColorDetector yellowDetector = new ColorDetector(C.YELLOW_COLOR, C.yellowLower, C.yellowUpper);
        ColorDetector greenDetector = new ColorDetector(C.GREEN_COLOR, C.greenLower, C.greenUpper);

        List<ColorDetector> detectors = new ArrayList<ColorDetector>();
        detectors.add(redDetector);
        detectors.add(yellowDetector);
        detectors.add(greenDetector);

        int numDetectors = detectors.size();
        
        visionThread = new Thread(() -> {

            while (true) { /// TODO: change condition later
                // System.out.println(usbCamera.getPath());
                long startTime = System.currentTimeMillis();

                if (cvSink.grabFrame(sourceMat) != 0) {

                    sourceMat.copyTo(tempMat);

                    // gaussian blur
                    Imgproc.GaussianBlur(sourceMat, sourceMat, C.gb, 0, 0);

                    outputStreamGB.putFrame(sourceMat);

                    BLACK_IMAGE.copyTo(outputMat);
                    BLACK_IMAGE.copyTo(redAreaCenter);

                    // color detection
                    for (int i = 0; i < numDetectors; i++) {
                        // for(int i = 0; i < 1; i++){
                        ColorDetector cd = detectors.get(i);
                        biggestColors[i] = cd.findRects(sourceMat);
                        if(i == 0){ // only do red color
                            outputStreamColorThresh.putFrame(cd.mask);
                            outputStreamContours.putFrame(cd.contourImage);
                            outputStreamContoursBB.putFrame(cd.contourBoundingBoxes);
                            outputStreamBB.putFrame(cd.boundingBoxes);

                            redAreaCenter.setTo(cd.color, cd.mask);
                            Imgproc.rectangle(redAreaCenter, biggestColors[i].tl(), biggestColors[i].br(), cd.color, 1);
                            Imgproc.circle(redAreaCenter, rectCenter(biggestColors[i]), 3, C.BLUE_COLOR, -1);

                            outputStreamArea.putFrame(redAreaCenter);
                        }
                        // System.out.println("found red");
                        outputMat.setTo(cd.color, cd.mask);
                        if (!C.VERBOSE) {
                            Imgproc.rectangle(outputMat, biggestColors[i].tl(), biggestColors[i].br(), cd.color, 1);
                            Imgproc.circle(outputMat, rectCenter(biggestColors[i]), 3, C.BLUE_COLOR, -1);
                            Imgproc.rectangle(tempMat, biggestColors[i].tl(), biggestColors[i].br(), cd.color, 1);
                            Imgproc.circle(tempMat, rectCenter(biggestColors[i]), 3, C.BLUE_COLOR, -1);
                            
                        }
                    }

                    biggestRed = biggestColors[0];
                    biggestYellow = biggestColors[1];
                    biggestGreen = biggestColors[2];

                    Imgproc.line(tempMat, centerTop, centerBottom, C.WHITE_COLOR, 1);
                    Imgproc.rectangle(tempMat, centerTopLeft, centerBottomRight, C.WHITE_COLOR, 1);

                    // output results
                    outputStream.putFrame(tempMat); // put processed image to smartdashboard
                    outputStreamFinal.putFrame(outputMat);
                    long endTime = System.currentTimeMillis();

                    System.out.println("Total execution time: " + (endTime - startTime));

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
