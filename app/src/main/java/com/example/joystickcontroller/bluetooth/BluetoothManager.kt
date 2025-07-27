package com.example.joystickcontroller.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import androidx.core.app.ActivityCompat
import java.io.IOException
import java.io.OutputStream
import java.util.*

class BluetoothManager(private val context: Context) {
    
    private val bluetoothAdapter: BluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private val uuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    
    var isConnected = false
        private set
    
    private var statusCallback: ((String) -> Unit)? = null
    private var deviceFoundCallback: ((BluetoothDevice) -> Unit)? = null
    
    fun setStatusCallback(callback: (String) -> Unit) {
        statusCallback = callback
    }
    
    fun setDeviceFoundCallback(callback: (BluetoothDevice) -> Unit) {
        deviceFoundCallback = callback
    }
    
    fun scanForDevices() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) 
            != PackageManager.PERMISSION_GRANTED) {
            statusCallback?.invoke("Bluetooth permissions not granted")
            return
        }
        
        if (bluetoothAdapter.isDiscovering) {
            bluetoothAdapter.cancelDiscovery()
        }
        
        statusCallback?.invoke("Scanning for devices...")
        bluetoothAdapter.startDiscovery()
    }
    
    fun connectToDevice(deviceAddress: String) {
        Thread {
            try {
                statusCallback?.invoke("Connecting...")
                
                if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) 
                    != PackageManager.PERMISSION_GRANTED) {
                    statusCallback?.invoke("Bluetooth permissions not granted")
                    return@Thread
                }
                
                val device: BluetoothDevice = bluetoothAdapter.getRemoteDevice(deviceAddress)
                bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
                bluetoothAdapter.cancelDiscovery()
                
                bluetoothSocket?.connect()
                outputStream = bluetoothSocket?.outputStream
                isConnected = true
                
                statusCallback?.invoke("Connected to ${device.name}")
            } catch (e: IOException) {
                statusCallback?.invoke("Connection failed: ${e.message}")
                
                // Try fallback connection method
                try {
                    val device: BluetoothDevice = bluetoothAdapter.getRemoteDevice(deviceAddress)
                    val method = device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                    bluetoothSocket = method.invoke(device, 1) as BluetoothSocket
                    bluetoothSocket?.connect()
                    outputStream = bluetoothSocket?.outputStream
                    isConnected = true
                    
                    statusCallback?.invoke("Connected (fallback)")
                } catch (e2: Exception) {
                    statusCallback?.invoke("Connection failed: ${e2.message}")
                    isConnected = false
                }
            }
        }.start()
    }
    
    fun sendData(data: String) {
        if (isConnected && outputStream != null) {
            try {
                outputStream?.write(data.toByteArray())
                outputStream?.flush()
            } catch (e: IOException) {
                statusCallback?.invoke("Send failed: ${e.message}")
                isConnected = false
            }
        }
    }
    
    fun disconnect() {
        try {
            isConnected = false
            outputStream?.close()
            bluetoothSocket?.close()
            statusCallback?.invoke("Disconnected")
        } catch (e: IOException) {
            statusCallback?.invoke("Disconnect error: ${e.message}")
        }
    }
    
    fun getPairedDevices(): Set<BluetoothDevice> {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) 
            != PackageManager.PERMISSION_GRANTED) {
            return emptySet()
        }
        return bluetoothAdapter.bondedDevices ?: emptySet()
    }
    
    fun createBluetoothReceiver(): BroadcastReceiver {
        return object : BroadcastReceiver() {
            override fun onReceive(context: Context, intent: Intent) {
                when (intent.action) {
                    BluetoothDevice.ACTION_FOUND -> {
                        val device: BluetoothDevice? = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                        device?.let { 
                            deviceFoundCallback?.invoke(it) 
                        }
                    }
                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                        statusCallback?.invoke("Scan completed")
                    }
                }
            }
        }
    }
    
    fun getIntentFilter(): IntentFilter {
        return IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
    }
}