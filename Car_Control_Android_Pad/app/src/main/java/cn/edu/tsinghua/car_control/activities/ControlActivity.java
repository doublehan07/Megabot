package cn.edu.tsinghua.car_control.activities;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.UUID;

import cn.edu.tsinghua.car_control.R;

public class ControlActivity extends Activity implements View.OnTouchListener, View.OnClickListener, SeekBar.OnSeekBarChangeListener, RadioGroup.OnCheckedChangeListener {
    private static final int COMMAND_STOP = 0;
    private static final int COMMAND_MOVE_FORWARD = 1;
    private static final int COMMAND_MOVE_BACKWARD = 2;
    private static final int COMMAND_TURN_LEFT = 3;
    private static final int COMMAND_TURN_RIGHT = 4;
    private static final int COMMAND_SEND_PERIOD = 100;
    private static final int MIN_DISTANCE_BETWEEN_CARS = 200;
    private static final int MAX_DISTANCE_BETWEEN_CARS = 10000;
    private static final String Serial_UUID = "00001101-0000-1000-8000-00805F9B34FB";
    private String mac = "";
    private Button reselect, reconnect;
    private TextView mac_tv, status, speed_bar_progress;
    private Button forward, backward, turn_left, turn_right;
    private SeekBar speed_bar;
    private int current_cmd = COMMAND_STOP;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothDevice mBluetoothDevice;
    private BluetoothSocket socket;
    private boolean isConnecting = false;
    private OutputStream outputStream;
    private RadioGroup shape_select_1, shape_select_2;
    private EditText x0, y0, x1, y1, x2, y2, x3, y3;
    private TextView size_bar_progress;
    private SeekBar size_bar;
    private Button send_shape;
    private boolean hasNewShape = true;
    private boolean shapeCorrect = false;
    private int currentCheckId = -1;
    private boolean isClearCallback = false;

    private TextWatcher coordinate_change_watcher = new TextWatcher() {
        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {

        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {

        }

        @Override
        public void afterTextChanged(Editable s) {
            if (currentCheckId == R.id.user_defined)
                updateShape();
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.control_layout);

        mac = getIntent().getStringExtra("MAC");
        reselect = (Button) findViewById(R.id.select_bt_device);
        reconnect = (Button) findViewById(R.id.connect_bt_device);
        mac_tv = (TextView) findViewById(R.id.bt_device_mac);
        status = (TextView) findViewById(R.id.bt_device_status);
        forward = (Button) findViewById(R.id.move_forward);
        backward = (Button) findViewById(R.id.move_backward);
        turn_left = (Button) findViewById(R.id.turn_left);
        turn_right = (Button) findViewById(R.id.turn_right);
        speed_bar = (SeekBar) findViewById(R.id.speed_bar);
        speed_bar_progress = (TextView) findViewById(R.id.speed_bar_progress);
        shape_select_1 = (RadioGroup) findViewById(R.id.shape_select_1);
        shape_select_2 = (RadioGroup) findViewById(R.id.shape_select_2);
        x0 = (EditText) findViewById(R.id.coordinate_0_x);
        y0 = (EditText) findViewById(R.id.coordinate_0_y);
        x1 = (EditText) findViewById(R.id.coordinate_1_x);
        y1 = (EditText) findViewById(R.id.coordinate_1_y);
        x2 = (EditText) findViewById(R.id.coordinate_2_x);
        y2 = (EditText) findViewById(R.id.coordinate_2_y);
        x3 = (EditText) findViewById(R.id.coordinate_3_x);
        y3 = (EditText) findViewById(R.id.coordinate_3_y);
        size_bar_progress = (TextView) findViewById(R.id.size_bar_progress);
        size_bar = (SeekBar) findViewById(R.id.size_bar);
        send_shape = (Button) findViewById(R.id.send_shape);

        reselect.setOnClickListener(this);
        reconnect.setOnClickListener(this);
        mac_tv.setText(mac);
        mac_tv.requestFocus();
        status.setText("未连接");
        forward.setOnTouchListener(this);
        backward.setOnTouchListener(this);
        turn_left.setOnTouchListener(this);
        turn_right.setOnTouchListener(this);
        speed_bar.setProgress(0);
        speed_bar.setOnSeekBarChangeListener(this);
        size_bar.setProgress(0);
        size_bar.setOnSeekBarChangeListener(this);
        send_shape.setOnClickListener(this);
        x0.setEnabled(false);
        y0.setEnabled(false);
        x1.setEnabled(false);
        x1.addTextChangedListener(coordinate_change_watcher);
        y1.setEnabled(false);
        y1.addTextChangedListener(coordinate_change_watcher);
        x2.setEnabled(false);
        x2.addTextChangedListener(coordinate_change_watcher);
        y2.setEnabled(false);
        y2.addTextChangedListener(coordinate_change_watcher);
        x3.setEnabled(false);
        x3.addTextChangedListener(coordinate_change_watcher);
        y3.setEnabled(false);
        y3.addTextChangedListener(coordinate_change_watcher);
        shape_select_1.setOnCheckedChangeListener(this);
        shape_select_2.setOnCheckedChangeListener(this);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        ConnectDevice();
    }

    private void updateShape() {
        int size = 300 + size_bar.getProgress();
        switch (currentCheckId) {
            case R.id.line:
                x1.setText(String.valueOf(-size));
                y1.setText("0");
                x2.setText(String.valueOf(-2 * size));
                y2.setText("0");
                x3.setText(String.valueOf(-3 * size));
                y3.setText("0");
                break;
            case R.id.rhombus:
                x1.setText(String.valueOf((int) (-size / Math.sqrt(2))));
                y1.setText(String.valueOf((int) (size / Math.sqrt(2))));
                x2.setText(String.valueOf((int) (-size / Math.sqrt(2))));
                y2.setText(String.valueOf((int) (-size / Math.sqrt(2))));
                x3.setText(String.valueOf((int) (-size * Math.sqrt(2))));
                y3.setText("0");
                break;
            case R.id.square:
                x1.setText("0");
                y1.setText(String.valueOf(size));
                x2.setText(String.valueOf(-size));
                y2.setText("0");
                x3.setText(String.valueOf(-size));
                y3.setText(String.valueOf(size));
                break;
            case R.id.large_triangular:
                x1.setText(String.valueOf((int) (size / Math.sqrt(3))));
                y1.setText("0");
                x2.setText(String.valueOf((int) (-size / Math.sqrt(3) / 2)));
                y2.setText(String.valueOf(-size / 2));
                x3.setText(String.valueOf((int) (-size / Math.sqrt(3) / 2)));
                y3.setText(String.valueOf(size / 2));
                break;
            case R.id.small_triangular:
                x1.setText(String.valueOf((int) (-size * Math.sqrt(3) / 2)));
                y1.setText(String.valueOf(size / 2));
                x2.setText(String.valueOf((int) (-size * Math.sqrt(3) / 2)));
                y2.setText(String.valueOf(-size / 2));
                x3.setText(String.valueOf((int) (-size * Math.sqrt(3) / 2)));
                y3.setText("0");
                break;
            case R.id.user_defined:
                break;
            default:
                break;
        }
        try {
            shapeCorrect = checkShape();
        } catch (Exception e) {
            shapeCorrect = false;
        }
    }

    private boolean checkShape() {
        double[] coors = new double[6];
        coors[0] = Double.parseDouble(x1.getText().toString());
        coors[1] = Double.parseDouble(y1.getText().toString());
        coors[2] = Double.parseDouble(x2.getText().toString());
        coors[3] = Double.parseDouble(y2.getText().toString());
        coors[4] = Double.parseDouble(x3.getText().toString());
        coors[5] = Double.parseDouble(y3.getText().toString());
        double tmp = Math.sqrt(Math.pow(coors[0], 2) + Math.pow(coors[1], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        tmp = Math.sqrt(Math.pow(coors[2], 2) + Math.pow(coors[3], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        tmp = Math.sqrt(Math.pow(coors[4], 2) + Math.pow(coors[5], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        tmp = Math.sqrt(Math.pow(coors[2] - coors[0], 2) + Math.pow(coors[3] - coors[1], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        tmp = Math.sqrt(Math.pow(coors[4] - coors[0], 2) + Math.pow(coors[5] - coors[1], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        tmp = Math.sqrt(Math.pow(coors[4] - coors[2], 2) + Math.pow(coors[5] - coors[3], 2));
        if (tmp < MIN_DISTANCE_BETWEEN_CARS || tmp > MAX_DISTANCE_BETWEEN_CARS)
            return false;
        return true;
    }

    private byte[] generateCmd() {
        byte[] cmd = new byte[4];
        cmd[0] = 0x7D;
        cmd[1] = (byte) current_cmd;
        cmd[3] = 0x7C;
        double speed = 0;
        if (current_cmd == COMMAND_MOVE_FORWARD || current_cmd == COMMAND_MOVE_BACKWARD) {
            speed = speed_bar.getProgress();
            speed = (speed > 10) ? 0.3 * (speed - 10) + 13 : 5;
        } else if (current_cmd == COMMAND_TURN_LEFT || current_cmd == COMMAND_TURN_RIGHT) {
            speed = speed_bar.getProgress();
            speed = (speed > 10) ? (speed - 10) / 6.0 + 10 : 5;
        } else if (current_cmd == COMMAND_STOP) {
            speed = 0;
        }
        cmd[2] = (byte) (int) speed;
        return cmd;
    }

    private byte[] generateShape() {
        byte[] shape = new byte[14];
        shape[0] = 0x7B;
        shape[13] = 0x7A;
        short tmp = Short.parseShort(x1.getText().toString());
        shape[1] = (byte) (tmp >> 8);
        shape[2] = (byte) tmp;
        tmp = Short.parseShort(y1.getText().toString());
        shape[3] = (byte) (tmp >> 8);
        shape[4] = (byte) tmp;
        tmp = Short.parseShort(x2.getText().toString());
        shape[5] = (byte) (tmp >> 8);
        shape[6] = (byte) tmp;
        tmp = Short.parseShort(y2.getText().toString());
        shape[7] = (byte) (tmp >> 8);
        shape[8] = (byte) tmp;
        tmp = Short.parseShort(x3.getText().toString());
        shape[9] = (byte) (tmp >> 8);
        shape[10] = (byte) tmp;
        tmp = Short.parseShort(y3.getText().toString());
        shape[11] = (byte) (tmp >> 8);
        shape[12] = (byte) tmp;
        return shape;
    }

    private void sendCommand() {
        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                if (outputStream == null || !isConnected()) {
                    status.setText("未连接");
                    outputStream = null;
                    return;
                }
                try {
                    outputStream.write(generateCmd());
                    outputStream.flush();
                    if (hasNewShape) {
                        if (shapeCorrect) {
                            outputStream.write(generateShape());
                            outputStream.flush();
                        }
                        hasNewShape = false;
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                    status.setText("未连接");
                    outputStream = null;
                    socket = null;
                }
                sendCommand();
            }
        }, COMMAND_SEND_PERIOD);
    }

    private boolean isConnected() {
        if (socket != null && socket.isConnected())
            return true;
        return false;
    }

    private void ConnectDevice() {
        if (isConnected() || isConnecting)
            return;
        isConnecting = true;
        status.setText("连接中");
        new ConnectThread().start();
    }

    private Handler handler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case 0:
                    isConnecting = false;
                    status.setText("未连接");
                    break;
                case 1:
                    isConnecting = false;
                    status.setText("已连接");
                    try {
                        outputStream = socket.getOutputStream();
                        sendCommand();
                    } catch (Exception e) {
                        status.setText("未连接");
                        outputStream = null;
                        e.printStackTrace();
                    }
                    break;
                default:
                    break;
            }
        }
    };

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        if (v.getId() == R.id.move_forward) {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                current_cmd = COMMAND_MOVE_FORWARD;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                if (current_cmd == COMMAND_MOVE_FORWARD)
                    current_cmd = COMMAND_STOP;
            }
        } else if (v.getId() == R.id.move_backward) {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                current_cmd = COMMAND_MOVE_BACKWARD;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                if (current_cmd == COMMAND_MOVE_BACKWARD)
                    current_cmd = COMMAND_STOP;
            }
        } else if (v.getId() == R.id.turn_left) {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                current_cmd = COMMAND_TURN_LEFT;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                if (current_cmd == COMMAND_TURN_LEFT)
                    current_cmd = COMMAND_STOP;
            }
        } else if (v.getId() == R.id.turn_right) {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                current_cmd = COMMAND_TURN_RIGHT;
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                if (current_cmd == COMMAND_TURN_RIGHT)
                    current_cmd = COMMAND_STOP;
            }
        }
        return false;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.select_bt_device:
                if (isConnected())
                    try {
                        socket.close();
                        status.setText("未连接");
                        socket = null;
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                Intent intent = new Intent(ControlActivity.this, BtSearchActivity.class);
                startActivity(intent);
                finish();
                break;
            case R.id.connect_bt_device:
                ConnectDevice();
                break;
            case R.id.send_shape:
                if (shapeCorrect)
                    hasNewShape = true;
                else
                    Toast.makeText(ControlActivity.this, "Shape is incorrect", Toast.LENGTH_SHORT).show();
                break;
            default:
                break;
        }
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        if (seekBar.getId() == R.id.speed_bar) {
            speed_bar_progress.setText(String.valueOf(progress));
        } else if (seekBar.getId() == R.id.size_bar) {
            size_bar_progress.setText(String.valueOf(progress + 200));
            updateShape();
        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onCheckedChanged(RadioGroup group, int checkedId) {
        if (isClearCallback) {
            if (checkedId == -1) {
                isClearCallback = false;
            }
            return;
        }
        if (group.getCheckedRadioButtonId() == R.id.user_defined) {
            x1.setEnabled(true);
            y1.setEnabled(true);
            x2.setEnabled(true);
            y2.setEnabled(true);
            x3.setEnabled(true);
            y3.setEnabled(true);
        } else {
            x1.setEnabled(false);
            y1.setEnabled(false);
            x2.setEnabled(false);
            y2.setEnabled(false);
            x3.setEnabled(false);
            y3.setEnabled(false);
        }
        currentCheckId = checkedId;
        if (group.getId() == R.id.shape_select_1) {
            isClearCallback = true;
            shape_select_2.clearCheck();
        } else {
            isClearCallback = true;
            shape_select_1.clearCheck();
        }
        updateShape();
    }

    private class ConnectThread extends Thread {
        private boolean connected = false;
        private int connectTime = 0;

        private void connectDevice() {
            try {
                if (mBluetoothDevice.getBondState() == BluetoothDevice.BOND_NONE) {
                    Method creMethod = BluetoothDevice.class.getMethod("createBond");
                    creMethod.invoke(mBluetoothDevice);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
            mBluetoothAdapter.cancelDiscovery();
            try {
                socket.connect();
                connected = true;
            } catch (Exception e) {
                connectTime++;
                connected = false;
                e.printStackTrace();
            }
        }

        @Override
        public void run() {
            mBluetoothDevice = mBluetoothAdapter.getRemoteDevice(mac);
            mBluetoothAdapter.cancelDiscovery();
            try {
                socket = mBluetoothDevice.createRfcommSocketToServiceRecord(UUID.fromString(Serial_UUID));
            } catch (Exception e) {
                e.printStackTrace();
            }
            connectTime = 0;
            connected = false;
            while (!connected && connectTime < 3) {
                connectDevice();
            }
            if (connected)
                handler.obtainMessage(1).sendToTarget();
            else
                handler.obtainMessage(0).sendToTarget();
        }
    }
}
