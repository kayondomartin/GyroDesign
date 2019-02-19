package com.example.gyrodesign;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.ParcelUuid;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.gyrodesign.LineWorks.Line;
import com.example.gyrodesign.LineWorks.LineSmoother;
import com.example.gyrodesign.LineWorks.Point;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private SensorManager sensorManager;

    private Handler mHandler = new Handler();

    FileOutputStream rssifile, gyrofile;

    private BluetoothManager btManager;
    private BluetoothAdapter btAdapter;
    private BluetoothLeScanner bleScanner;
    private static final int MY_PERMISSION_REQUEST_READ_CONTACTS = 1;

    private static final ParcelUuid EDDYSTONE_SERVICE_UUID = ParcelUuid.fromString("0000FEAA-0000-1000-8000-00805F9B34FB");
    private static final String DeviceAddress = "E1:6E:58:BC:F1:82";

    private static final ScanFilter EDDYSTONE_SCAN_FILTER = new ScanFilter.Builder()
            .setServiceUuid(EDDYSTONE_SERVICE_UUID)
            .setDeviceAddress(DeviceAddress)
            .build();

    private ScanSettings SCAN_SETTINGS =
            new ScanSettings.Builder()
                    .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                    .setReportDelay(0)
                    .build();

    private List<ScanFilter> SCAN_FILTERS = buildScanFilters();


    private static List<ScanFilter> buildScanFilters(){
        List<ScanFilter> scanFilters = new ArrayList<>();
        scanFilters.add(EDDYSTONE_SCAN_FILTER);
        return scanFilters;
    }

    private Button controlButton;
    private ImageView directionIconImage;
    private TextView processTextView;
    private TextView addressTextView;

    private List<Point> rssiList;
    private List<Point> gyroList;

    private float[] gyro = new float[3];
    private float[] gyroMatrix = new float[9];
    private float[] gyroOrientation = new float[3];
    private float[] rotationMatrix = new float[9];
    private float[] magnet = new float[3];
    private float[] accel = new float[3];
    private float[] accMagOrientation = new float[3];
    private float[] fusedOrientation = new float[3];

    public static final float EPSILON = 0.000000001f;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private float timestamp;
    private boolean initState = true;

    public static final int TIME_CONSTANT = 50;
    public static final float FILTER_COEFFICIENT = 0.98f;
    private Timer fuseTimer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);


        btManager = (BluetoothManager) getSystemService(BLUETOOTH_SERVICE);
        btAdapter = btManager.getAdapter();
        bleScanner = btAdapter.getBluetoothLeScanner();

        rssiList = new ArrayList<>();
        gyroList = new ArrayList<>();

        if(btAdapter == null || bleScanner == null){
            Toast.makeText(this,"Either bluetooth or BLE not supported!",Toast.LENGTH_SHORT);
            finish();
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_DENIED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},
                    MY_PERMISSION_REQUEST_READ_CONTACTS);
        }

        controlButton = findViewById(R.id.control_button);
        controlButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(!isScanning){
                    startScan();
                    fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),
                            1000, TIME_CONSTANT);
                }else{
                    stopScan();
                }
            }
        });

    }
    @Override
    public void onSensorChanged(SensorEvent event) {

        switch(event.sensor.getType()){
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values,0,accel,0,3);
                calculateAccMagOrientation();
                break;
            case Sensor.TYPE_GYROSCOPE:
                gyroFunction(event);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values,0,magnet,0,3);
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    double initialTime = -10;
    Point lastUpdate = null;
    private ScanCallback scanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            super.onScanResult(callbackType, result);

            int rssi = result.getRssi();

            if(lastUpdate == null){
                lastUpdate = new Point(0.0, rssi);
                initialTime = System.currentTimeMillis();
                rssiList.add(new Point(lastUpdate));
            }else{
                double time = (System.currentTimeMillis()-initialTime)/1000.0;
                lastUpdate = new Point(time, rssi);
                rssiList.add(new Point(lastUpdate));
            }
        }
    };

    private void initListeners(){
        sensorManager.registerListener(this,sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),100000);
        sensorManager.registerListener(this,sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),100000);
        sensorManager.registerListener(this,sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),100000);
        bleScanner.startScan(SCAN_FILTERS,SCAN_SETTINGS,scanCallback);
        isScanning = true;
    }

    private static boolean isScanning = false;

    private void startScan(){
        fuseTimer = new Timer();
        initListeners();
        try {
            rssifile = openFileOutput("rssiFile.txt", MODE_PRIVATE);
            gyrofile = openFileOutput("gyrofile.txt", MODE_PRIVATE);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }

    private void deInitListeners(){
        sensorManager.unregisterListener(this);
        bleScanner.stopScan(scanCallback);
        isScanning = false;
        fuseTimer.cancel();
    }

    private void stopScan(){
        deInitListeners();
        mHandler.removeCallbacks(saveAngle);
        Log.e(MainActivity.class.getSimpleName(),"GyroSize: "+gyroList.size());
        List<Line> smoothedRSSI = LineSmoother.smoothLine(rssiList);
        List<Line> smoothedGyro = LineSmoother.smoothLine(gyroList);



        double start = gyroList.get(0).getX() > rssiList.get(0).getX()? gyroList.get(0).getX(): rssiList.get(0).getX();

        List<Point> sampledRSSI = LineSmoother.sample(smoothedRSSI,start,0.05,smoothedRSSI.get(smoothedRSSI.size()-1).getPoint2().getX());
        List<Point> sampledGyro = LineSmoother.sample(smoothedGyro,start,0.05,smoothedGyro.get(smoothedGyro.size()-1).getPoint2().getX());

        int rssiSize = sampledRSSI.size();
        int gyroSize = sampledGyro.size();
        try {
            for (int i = 0; i < rssiSize; i++) {
                String record = sampledRSSI.get(i).getX() + "\t" + sampledRSSI.get(i).getY()+"\n";
                rssifile.write(record.getBytes());
            }

            for (int i=0;i<gyroSize;i++){
                String record = sampledGyro.get(i).getX() + "\t" + sampledGyro.get(i).getY() + "\n";
                gyrofile.write(record.getBytes());
            }
        }catch(IOException e){
            e.printStackTrace();
        }finally {
            try{
                rssifile.close();
                gyrofile.close();
            }catch(IOException e){
                e.printStackTrace();
            }
        }
        sampledGyro = LineSmoother.standardize(sampledGyro);
        sampledRSSI = LineSmoother.standardize(sampledRSSI);


        int size = sampledGyro.size() > sampledRSSI.size()? sampledRSSI.size() : sampledGyro.size();
        try {
            double corr = LineSmoother.corrCoeff(sampledGyro.subList(0,size),sampledRSSI.subList(0,size));
            Log.e(MainActivity.class.getSimpleName(),"Corr: "+corr);
        } catch (LineSmoother.NonEqualException e) {
            e.printStackTrace();
        }

        rssiList.clear();
        gyroList.clear();
        lastUpdate = null;
        initialTime = -10;
    }

    private double square(double a){
        return a*a;
    }

    private float dotProduct(float[] a){
        float sum = 0;
        int i;
        int length = a.length;
        for(i=0; i<length;i++){
            sum+=square(a[i]);
        }

        return sum;
    }

    private void getRotationVectorFromGyro(float[] gyroValues, float[] deltaRotationVector, float timeFactor){

        float[] normValues = new float[3];

        float omegaMagnitude = (float)Math.sqrt(dotProduct(gyroValues));

        if(omegaMagnitude > EPSILON){
            normValues[0] = gyroValues[0]/omegaMagnitude;
            normValues[1] = gyroValues[1]/omegaMagnitude;
            normValues[2] = gyroValues[2]/omegaMagnitude;
        }

        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    public void gyroFunction(SensorEvent event){

        if(accMagOrientation == null){
            return;
        }

        if(initState){
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix,test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        float[] deltaVector = new float[4];

        if(timestamp != 0){
            final float dT = (event.timestamp - timestamp)*NS2S;
            System.arraycopy(event.values,0,gyro,0,3);
            getRotationVectorFromGyro(gyro,deltaVector,dT/2.0f);
        }

        timestamp = event.timestamp;
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix,deltaVector);

        gyroMatrix = matrixMultiplication(gyroMatrix,deltaMatrix);

        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
        double cosine = Math.cos(gyroOrientation[1]*180/Math.PI);
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;



            // azimuth
            if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]);
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
            }

            // pitch
            if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]);
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
            }

            // roll
            if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]);
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
            }

            // overwrite gyro matrix and orientation with fused orientation
            // to comensate gyro drift
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);


            // update sensor output in GUI
            mHandler.post(saveAngle);
        }

    }
    private Runnable saveAngle = new Runnable() {
        @Override
        public void run() {
            long time = System.currentTimeMillis();
            double angle = gyroOrientation[1]*180/Math.PI;
            double hinge = gyroOrientation[2];
            /*if(hinge>0){
                angle = -angle;
            }else{
                angle = 180.0 + angle;
            }
            */


            if(initialTime < 0){
                gyroList.add(new Point(0.0,angle));
                initialTime = time;
            }else{
                gyroList.add(new Point((time-initialTime)/1000.0,angle));
            }

            Log.e(MainActivity.class.getSimpleName(),"["+(time-initialTime)/1000.0+","+hinge+"]");
        }
    };
}
