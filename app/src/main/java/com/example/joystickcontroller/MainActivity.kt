package com.example.joystickcontroller

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.content.BroadcastReceiver
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.example.joystickcontroller.bluetooth.BluetoothManager
import com.example.joystickcontroller.ui.GameButton
import com.example.joystickcontroller.ui.JoystickView

class MainActivity : AppCompatActivity() {

    private lateinit var bluetoothManager: BluetoothManager
    private lateinit var bluetoothReceiver: BroadcastReceiver

    // UI Components
    private lateinit var statusText: TextView
    private lateinit var scanButton: Button
    private lateinit var connectButton: Button
    private lateinit var deviceSpinner: Spinner
    private lateinit var joystickView: JoystickView
    private lateinit var joystickValueText: TextView

    // Game Buttons
    private lateinit var triangleButton: GameButton
    private lateinit var squareButton: GameButton
    private lateinit var circleButton: GameButton
    private lateinit var xButton: GameButton

    // Device Management
    private val deviceList = mutableListOf<BluetoothDevice>()
    private val deviceNames = mutableListOf<String>()
    private lateinit var deviceAdapter: ArrayAdapter<String>
    private var selectedDeviceAddress: String? = null

    //--- MODIFIKASI DIMULAI DI SINI ---//

    // 1. Variabel untuk menyimpan state kontrol terkini
    private var joystickX = 0f
    private var joystickY = 0f
    private var trianglePressed = false
    private var squarePressed = false
    private var circlePressed = false
    private var xPressed = false

    // 2. Handler dan Runnable untuk loop pengiriman data 50Hz
    private val dataHandler = Handler(Looper.getMainLooper())
    private var isSendingData = false

    private val dataSenderRunnable = object : Runnable {
        override fun run() {
            // Pastikan kita harus mengirim data (terhubung dan diaktifkan)
            if (bluetoothManager.isConnected && isSendingData) {
                // Buat satu frame data yang berisi semua state
                // Format: "joy:x,y;i:s;b:s;m:s;a:s\n" (s = 1 untuk ditekan, 0 untuk dilepas)
                val dataFrame = "joy:${"%.3f".format(joystickX)},${"%.3f".format(joystickY)};" +
                        "i:${if (trianglePressed) 1 else 0};" +
                        "b:${if (squarePressed) 1 else 0};" +
                        "m:${if (circlePressed) 1 else 0};" +
                        "a:${if (xPressed) 1 else 0}\n"

                // Kirim frame data
                bluetoothManager.sendData(dataFrame)

                // Jadwalkan pengiriman berikutnya dalam 20ms (50Hz)
                dataHandler.postDelayed(this, 20)
            }
        }
    }

    // 3. Fungsi untuk memulai dan menghentikan loop pengiriman
    private fun setSendingState(shouldSend: Boolean) {
        if (shouldSend && !isSendingData) {
            isSendingData = true
            dataHandler.post(dataSenderRunnable)
        } else if (!shouldSend) {
            isSendingData = false
            dataHandler.removeCallbacks(dataSenderRunnable)
        }
    }

    //--- MODIFIKASI SELESAI DI SINI ---//

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        hideSystemUI()
        initViews()
        setupBluetooth()
        setupJoystick()
        setupGameButtons()
        setupDeviceSpinner()
        requestPermissions()
    }

    private fun hideSystemUI() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.insetsController?.let { controller ->
                controller.hide(WindowInsets.Type.statusBars() or WindowInsets.Type.navigationBars())
                controller.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                    View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
                            View.SYSTEM_UI_FLAG_FULLSCREEN or
                            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                    )
        }
    }

    private fun initViews() {
        statusText = findViewById(R.id.statusText)
        scanButton = findViewById(R.id.scanButton)
        connectButton = findViewById(R.id.connectButton)
        deviceSpinner = findViewById(R.id.deviceSpinner)
        joystickView = findViewById(R.id.joystickView)
        joystickValueText = findViewById(R.id.joystickValueText)

        triangleButton = findViewById(R.id.triangleButton)
        squareButton = findViewById(R.id.squareButton)
        circleButton = findViewById(R.id.circleButton)
        xButton = findViewById(R.id.xButton)

        setControlsEnabled(false)
    }

    private fun setupBluetooth() {
        bluetoothManager = BluetoothManager(this)

        bluetoothManager.setStatusCallback { status ->
            runOnUiThread {
                statusText.text = status
                updateConnectionButton()

                val isConnected = bluetoothManager.isConnected
                setControlsEnabled(isConnected)

                // MODIFIKASI: Mulai atau hentikan loop pengiriman data berdasarkan status koneksi
                setSendingState(isConnected)

                if (isConnected) {
                    statusText.setTextColor(resources.getColor(android.R.color.holo_green_light, null))
                } else {
                    statusText.setTextColor(resources.getColor(android.R.color.white, null))
                }
            }
        }

        bluetoothManager.setDeviceFoundCallback { device ->
            runOnUiThread {
                if (!deviceList.any { it.address == device.address }) {
                    deviceList.add(device)
                    deviceNames.add("${device.name ?: "Unknown"} (${device.address})")
                    deviceAdapter.notifyDataSetChanged()
                }
            }
        }

        bluetoothReceiver = bluetoothManager.createBluetoothReceiver()
        registerReceiver(bluetoothReceiver, bluetoothManager.getIntentFilter())

        scanButton.setOnClickListener { startScan() }
        connectButton.setOnClickListener { toggleConnection() }
    }

    private fun setupJoystick() {
        joystickView.setJoystickCallback { x, y ->
            // MODIFIKASI: HANYA perbarui variabel state
            joystickX = x
            joystickY = y

            // Anda masih bisa memperbarui UI di sini jika mau
            joystickValueText.text = "X: ${"%.2f".format(x)}, Y: ${"%.2f".format(y)}"
        }
    }

    private fun setupGameButtons() {
        // Triangle - Green
        triangleButton.setText("IDLE")
        triangleButton.setButtonColor(Color.rgb(0, 150, 0))
        triangleButton.setTextSize(50f)
        triangleButton.setButtonCallback { pressed ->
            // MODIFIKASI: HANYA perbarui variabel state
            trianglePressed = pressed
        }

        // Square - Pink
        squareButton.setText("BOOST")
        squareButton.setButtonColor(Color.rgb(255, 100, 150))
        squareButton.setTextSize(50f)
        squareButton.setButtonCallback { pressed ->
            // MODIFIKASI: HANYA perbarui variabel state
            squarePressed = pressed
        }

        // Circle - Red
        circleButton.setText("MANUAL")
        circleButton.setButtonColor(Color.rgb(200, 50, 50))
        circleButton.setTextSize(45f)
        circleButton.setButtonCallback { pressed ->
            // MODIFIKASI: HANYA perbarui variabel state
            circlePressed = pressed
        }

        // X - Blue
        xButton.setText("AUTO")
        xButton.setButtonColor(Color.rgb(50, 100, 200))
        xButton.setTextSize(50f)
        xButton.setButtonCallback { pressed ->
            // MODIFIKASI: HANYA perbarui variabel state
            xPressed = pressed
        }
    }

    private fun setupDeviceSpinner() {
        deviceNames.add("Select device...")
        deviceAdapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, deviceNames)
        deviceAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        deviceSpinner.adapter = deviceAdapter

        deviceSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>?, view: android.view.View?, position: Int, id: Long) {
                if (position > 0 && position <= deviceList.size) {
                    selectedDeviceAddress = deviceList[position - 1].address
                    val deviceName = deviceList[position - 1].name ?: "Unknown"
                    statusText.text = "Selected: $deviceName"
                }
            }
            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }
    }

    private fun requestPermissions() {
        val permissions = arrayOf(
            Manifest.permission.BLUETOOTH_CONNECT,
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        ActivityCompat.requestPermissions(this, permissions, 1)
    }

    private fun startScan() {
        deviceList.clear()
        deviceNames.clear()
        deviceNames.add("Select device...")

        bluetoothManager.getPairedDevices().forEach { device ->
            if (!deviceList.contains(device)) {
                deviceList.add(device)
                deviceNames.add("${device.name ?: "Unknown"} (${device.address}) [Paired]")
            }
        }
        deviceAdapter.notifyDataSetChanged()

        bluetoothManager.scanForDevices()
        scanButton.text = "Scanning..."
        scanButton.isEnabled = false

        scanButton.postDelayed({
            scanButton.text = "Scan"
            scanButton.isEnabled = true
        }, 10000)
    }

    private fun toggleConnection() {
        if (bluetoothManager.isConnected) {
            bluetoothManager.disconnect()
        } else {
            selectedDeviceAddress?.let { address ->
                bluetoothManager.connectToDevice(address)
            } ?: run {
                statusText.text = "Please select a device first"
                Toast.makeText(this, "Please select a device first", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun updateConnectionButton() {
        connectButton.text = if (bluetoothManager.isConnected) "Disconnect" else "Connect"
        connectButton.backgroundTintList = if (bluetoothManager.isConnected) {
            resources.getColorStateList(android.R.color.holo_red_dark, null)
        } else {
            resources.getColorStateList(android.R.color.holo_green_dark, null)
        }
    }

    private fun setControlsEnabled(enabled: Boolean) {
        joystickView.isEnabled = enabled
        triangleButton.isEnabled = enabled
        squareButton.isEnabled = enabled
        circleButton.isEnabled = enabled
        xButton.isEnabled = enabled

        val alpha = if (enabled) 1.0f else 0.5f
        joystickView.alpha = alpha
        triangleButton.alpha = alpha
        squareButton.alpha = alpha
        circleButton.alpha = alpha
        xButton.alpha = alpha
    }

    override fun onDestroy() {
        super.onDestroy()
        // MODIFIKASI: Pastikan loop berhenti saat aplikasi ditutup
        setSendingState(false)
        try {
            unregisterReceiver(bluetoothReceiver)
            bluetoothManager.disconnect()
        } catch (e: Exception) {}
    }

    override fun onResume() {
        super.onResume()
        hideSystemUI()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            hideSystemUI()
        }
    }
}