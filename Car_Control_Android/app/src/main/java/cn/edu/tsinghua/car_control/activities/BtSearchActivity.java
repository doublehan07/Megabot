package cn.edu.tsinghua.car_control.activities;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import cn.edu.tsinghua.car_control.R;

public class BtSearchActivity extends Activity implements View.OnClickListener {
    private TextView bt_status;
    private Button open_bt, close_bt, bt_start_scan, bt_stop_scan;
    private ListView bt_device_list;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothReceiver receiver;
    private ArrayAdapter<String> adapter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.bt_search_layout);

        bt_status = (TextView) findViewById(R.id.bt_status);
        open_bt = (Button) findViewById(R.id.open_bt);
        close_bt = (Button) findViewById(R.id.close_bt);
        bt_start_scan = (Button) findViewById(R.id.bt_start_scan);
        bt_stop_scan = (Button) findViewById(R.id.bt_stop_scan);
        bt_device_list = (ListView) findViewById(R.id.bt_device_list);

        open_bt.setOnClickListener(this);
        close_bt.setOnClickListener(this);
        bt_start_scan.setOnClickListener(this);
        bt_stop_scan.setOnClickListener(this);
        bt_device_list.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                String device = adapter.getItem(position);
                String[] strs = device.split("-| ");
                String mac = "";
                if (strs[0].contains(":"))
                    mac = strs[0];
                else if (strs[1].contains(":"))
                    mac = strs[1];
                else if (strs[2].contains(";"))
                    mac = strs[2];
                if(mac.equals(""))
                    return;
                if (mBluetoothAdapter.isDiscovering())
                    mBluetoothAdapter.cancelDiscovery();
                Intent intent = new Intent(BtSearchActivity.this, ControlActivity.class);
                intent.putExtra("MAC", mac);
                startActivity(intent);
                finish();
            }
        });

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        UpdateBtStatus();
        receiver = new BluetoothReceiver();
        IntentFilter intents = new IntentFilter();
        intents.addAction(BluetoothDevice.ACTION_FOUND);
        intents.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
        intents.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        intents.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(receiver, intents);
    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(receiver);
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.open_bt:
                if (!mBluetoothAdapter.isEnabled()) {
                    if (mBluetoothAdapter.enable()) {
                        Toast.makeText(BtSearchActivity.this, "Bluetooth is opened!", Toast.LENGTH_SHORT).show();
                        bt_status.setText("Bluetooth On");
                    }
                }
                break;
            case R.id.close_bt:
                if (mBluetoothAdapter.isEnabled()) {
                    if (mBluetoothAdapter.disable()) {
                        Toast.makeText(BtSearchActivity.this, "Bluetooth is closed!", Toast.LENGTH_SHORT).show();
                        bt_status.setText("Bluetooth Off");
                    }
                }
                break;
            case R.id.bt_start_scan:
                if(!mBluetoothAdapter.isEnabled())
                {
                    Toast.makeText(BtSearchActivity.this, "Please open bluetooth first", Toast.LENGTH_SHORT).show();
                }
                if (mBluetoothAdapter.isDiscovering()) {
                    mBluetoothAdapter.cancelDiscovery();
                }
                mBluetoothAdapter.startDiscovery();
                break;
            case R.id.bt_stop_scan:
                if (mBluetoothAdapter.isDiscovering()) {
                    mBluetoothAdapter.cancelDiscovery();
                }
                break;
            default:
                break;
        }
    }

    private void UpdateBtStatus() {
        if (mBluetoothAdapter == null)
            return;
        if (!mBluetoothAdapter.isEnabled())
            bt_status.setText("Bluetooth Off");
        else if (mBluetoothAdapter.isDiscovering())
            bt_status.setText("Bluetooth Scanning");
        else
            bt_status.setText("Bluetooth On");
    }

    private String ParseBtDevice(BluetoothDevice device) {
        StringBuilder sb = new StringBuilder();
        if (device.getName() != null)
            sb.append(device.getName()).append("-").append(device.getAddress());
        else
            sb.append(device.getAddress());
        if (device.getBondState() == device.BOND_BONDED)
            sb.append(" (Bonded)");
        else if (device.getBondState() == device.BOND_BONDING)
            sb.append(" (Bonding)");
        return sb.toString();
    }

    private class BluetoothReceiver extends BroadcastReceiver {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(BluetoothDevice.ACTION_FOUND)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (adapter == null) {
                    adapter = new ArrayAdapter<>(BtSearchActivity.this, android.R.layout.simple_list_item_1);
                    bt_device_list.setAdapter(adapter);
                }
                adapter.add(ParseBtDevice(device));
            } else if (intent.getAction().equals(BluetoothAdapter.ACTION_DISCOVERY_STARTED)) {
                Toast.makeText(BtSearchActivity.this, "Start Scanning", Toast.LENGTH_SHORT).show();
                UpdateBtStatus();
                if (adapter != null)
                    adapter.clear();
            } else if (intent.getAction().equals(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)) {
                Toast.makeText(BtSearchActivity.this, "Finish Scanning", Toast.LENGTH_SHORT).show();
                UpdateBtStatus();
            } else if (intent.getAction().equals(BluetoothAdapter.ACTION_STATE_CHANGED)) {
                UpdateBtStatus();
            }
        }
    }
}
