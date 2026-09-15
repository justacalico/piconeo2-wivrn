package org.meumeu.wivrn.neo2.pvr;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.KeyEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.WindowManager;
import java.util.List;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

import com.unity3d.player.UnityPlayer;

import com.picovr.picovrlib.cvcontrollerclient.ControllerClient;
import com.picovr.picovrlib.cvcontrollerclient.BindControllerCallback;

/**
 * Host activity (Unity-emulation approach).
 *
 * The Pico SDK obtains the render Surface from the activity's `mUnityPlayer`
 * field (a ViewGroup whose first child is the SurfaceView). We provide exactly
 * that. Our native render thread renders the eyes into offscreen FBOs; the SDK's
 * own TimeWarp thread owns the window surface (pulled from mUnityPlayer) and
 * presents to the HMD.
 */
public class MainActivity extends Activity implements SurfaceHolder.Callback {

    private static final String TAG = "wivrn";

    static {
        System.loadLibrary("tracking_module");
        System.loadLibrary("native");
        System.loadLibrary("Pvr_UnitySDK");
        System.loadLibrary("p2wivrn");
    }

    // IMPORTANT: the SDK looks up this exact field name/type on the activity.
    // Its surfaceView is what the SDK's TimeWarp thread presents to (we let it
    // black that one out harmlessly, BEHIND our own render surface).
    public UnityPlayer mUnityPlayer;

    // Our own render surface, composited ON TOP of the SDK's. We self-present
    // video here; the SDK warp thread never touches it, so no surface conflict.
    private SurfaceView mRenderView;

    private native void nativeStart(Activity activity);

    private void extractAsset(String assetPath, File outFile) {
        if (outFile.exists()) return;
        outFile.getParentFile().mkdirs();
        try (InputStream in = getAssets().open(assetPath);
             OutputStream out = new FileOutputStream(outFile)) {
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        } catch (Exception e) {
            Log.e(TAG, "extractAsset " + assetPath, e);
        }
    }
    private void extractControllerModels() {
        File dir = getFilesDir();
        extractAsset("controller/r.obj", new File(dir, "controller/r.obj"));
        extractAsset("controller/controller2s.obj", new File(dir, "controller/controller2s.obj"));
        extractAsset("controller/controller2s_idle.png", new File(dir, "controller/controller2s_idle.png"));
        extractAsset("controller/controller2s_app.png", new File(dir, "controller/controller2s_app.png"));
        extractAsset("controller/controller2s_home.png", new File(dir, "controller/controller2s_home.png"));
        extractAsset("controller/controller2s_touchpad.png", new File(dir, "controller/controller2s_touchpad.png"));
        extractAsset("controller/controller2s_trigger.png", new File(dir, "controller/controller2s_trigger.png"));
    }
    private native void nativeSurfaceChanged(android.view.Surface surface);
    private native void nativeSurfaceDestroyed();
    private native void nativeStop();
    private native void nativeSetSleep(boolean sleep);
    // Test hook: arm the low-battery popup at a given percentage (see mTestReceiver).
    private native void nativeTestBatteryWarn(int pct);
    private native void nativeKeyEvent(int keyCode, boolean down);
    private native void nativeControllerState(int hand, int conn,
            float[] sensor, float[] angVel, int[] keys);
    // Latest head pose (raw sensor frame: qx,qy,qz,qw,px,py,pz) from native, fed
    // into the CV service so controller poses come back in the head's frame.
    private native void nativeGetHeadData(float[] out);
    private final float[] mHeadData = new float[7];
    // Server-driven haptics: native parks a pending rumble per hand; this poller
    // drains it and drives the Pico CV2 wand. out[0]=amplitude(0..1), out[1]=ms.
    private native boolean nativeDrainHaptic(int hand, float[] out);
    private final float[] mHaptic = new float[2];

    // Ported WiVRn lobby UI (matches pico_oxr).
    private native void nativeSetServerList(String[] names, String[] hosts, int[] ports, boolean[] tcpOnly, boolean[] discovered, boolean[] autoconnect);
    public native void nativeSetFov(float fovDeg);
    private native boolean nativeReady();
    private native void nativeConnect(String hostname, int port, boolean tcpOnly);
    private native void nativeDisconnect();
    private native void nativeSetPin(String pin);
    public native void nativeSetBitrate(int bitrateMbps);
    public native void nativeSetPassthrough(boolean enabled);
    private native void nativeSetIpd(float ipdMm);
    private native void nativeSetMicrophone(boolean enabled);
    private native void nativeSetStreamResolution(int width, int height);
    private native void nativeSetRenderResolution(int width, int height);
    public native void nativeRequestAppList();
    public native void nativeStartApp(String appId);
    public native void nativeRequestRunningApps();
    public native void nativeSetActiveApp(int appId);
    public native void nativeStopApp(int appId);
    public native void nativeRecenter();
    public native void nativeSetBrightness(float frac);
    public native void nativeSetCtrlVibration(float strength);
    public native void nativeSetEyeFoveation(boolean enabled);
    public native void nativeSetFoveationEnabled(boolean enabled);
    public native void nativeSetEyeDebug(boolean enabled);
    public native void nativeSetDiagHud(int mode);
    public native boolean nativeIsEyeSupported();

    private ServerDiscovery serverDiscovery;
    private volatile boolean mServerSyncRunning = false;
    private Thread mServerSyncThread;

    // Android View-based VR UI
    private VrUiPanel mVrUiPanel;


    // Pending WiVRn dashboard connection (flushed once nativeReady() is true).
    private String pendingHost;
    private int pendingPort;
    private boolean pendingTcpOnly;
    private String pendingPin;
    private boolean hasPendingConnection = false;
    private long lastConnectFlushMs = 0;
    private static final long CONNECT_DEBOUNCE_MS = 2000;
    private volatile boolean nativeConnecting = false;

    // Pico CV controller service: bound once, then polled on a background thread
    // and pushed to native. headDof/handDof = 1,1 requests 6DoF.
    private volatile boolean mCtrlRunning = false;
    private Thread mCtrlThread;
    // The Pico CV service assigns controller handedness at the moment
    // startControllerThread() runs. On a cold start, calling it in bindSuccess fires
    // before BOTH controllers have connected, so the service binds handedness with
    // only one controller present -> left maps to the right controller, the other
    // hand is dead, and the stick reads uninitialized (diagonal) garbage. Re-opening
    // the app "fixes" it only because both are connected by the next start.
    //
    // Rather than restart the thread to re-bind (fragile: a 60ms stop/start often
    // doesn't take, and driving it off lifecycle hooks loops forever), we DEFER the
    // first and ONLY startControllerThread until both controllers have been reported
    // connected CONTINUOUSLY for a settle period. The service reports "connected"
    // instantly on bind (stale/cached state), starting the 6DoF thread that early
    // binds handedness before the controller link has truly re-established (worst on
    // the first launch after an update, when the service was still attached to the
    // just-killed process), which is the swap. Waiting for the link to settle first
    // lets it bind correctly. Tracked in the poll loop; started exactly once.
    private volatile boolean mConn0 = false;   // controller 0 connected
    private volatile boolean mConn1 = false;   // controller 1 connected
    private volatile boolean mCtrlThreadStarted = false;
    private int diagTick = 0;  // diagnostic log counter for controller sensor status
    private long mBothConnSinceMs = 0;         // when both first became connected (0 = not yet)
    private static final long CTRL_SETTLE_MS = 1500;  // both-connected must hold this long
    // Set when our surface is torn down; consumed when it comes back. A full-screen
    // takeover (e.g. the double-click-Home shortcut panel com.pvr.shortcut, a Unity VR
    // Activity) destroys our surface AND reconfigures the SHARED CV controller service
    // for its own session (it re-sets unity version to 2.9.0.0 + re-anchors the
    // controller link). When it exits our surface is recreated and render recovers,
    // but the CV service is left on the shortcut's head-fusion/recenter reference, so
    // our hands stay locked to a stale head frame (they swing opposite to head
    // rotation, decoupled from world). Home/back fixes it only because it re-runs our
    // bind. We replicate that recovery on the surface destroy->recreate edge, a clean
    // signal (unlike onResume, which fires on every focus change and loops). Re-claim
    // settle is short: both controllers are already healthy here (the shortcut kept
    // them connected), so this is NOT the confused cold-start state.
    private volatile boolean mSurfaceWasDestroyed = false;
    private static final long CTRL_RECLAIM_SETTLE_MS = 300;

    // True only while the activity is in the foreground (between onResume and
    // onPause). When backgrounded (Home pressed / another app on top) we stop
    // forwarding controller tracking + input to the server and stop driving
    // haptics, so the emulated controllers don't keep moving/buzzing in SteamVR
    // while the user is away. The poll loop keeps running so the CV link stays
    // warm and resumes instantly on return.
    private volatile boolean mForeground = false;
    private volatile long mSettleMs = CTRL_SETTLE_MS;  // active settle window (longer cold, short on re-claim)
    // Reported to the CV service so it flips callunityversion=true and assigns
    // handedness from real per-controller state (see bindSuccess). Matches the
    // version the system Unity/CVControllerManager reports on this firmware.
    private static final String CTRL_UNITY_VERSION = "2.8.6.9";

    // Held for the app's lifetime so the Wi-Fi link stays reliable for streaming:
    //  - MulticastLock: WiVRn discovers the PC server via mDNS/broadcast; without
    //    this, Android filters multicast/broadcast packets and discovery can stall.
    //  - WifiLock(HIGH_PERF): keeps the Wi-Fi radio out of power-save so latency
    //    doesn't spike / the link doesn't drop when the screen state changes.
    private WifiManager.MulticastLock mMulticastLock;
    private WifiManager.WifiLock mWifiLock;

    // Proximity power-sleep: when the headset is taken OFF the head (proximity
    // reads "far") for PROX_SLEEP_MS, pause the stream + release the screen-on lock
    // so the panel can sleep, big power saving while it sits idle. Donning it
    // (proximity "near") cancels the timer / wakes it back up. We MUST drop
    // FLAG_KEEP_SCREEN_ON here: that flag (held for streaming) is exactly what stops
    // the system from auto-sleeping the panel when the headset is doffed.
    private SensorManager mSensorManager;
    private Sensor mProximity;
    private final Handler mMainHandler = new Handler(Looper.getMainLooper());
    private static final long PROX_SLEEP_MS = 60_000L;   // 1 minute off-head -> sleep
    private boolean mAsleep = false;
    private final Runnable mSleepRunnable = new Runnable() {
        @Override public void run() {
            if (mAsleep) return;
            mAsleep = true;
            Log.i(TAG, "proximity: off-head " + (PROX_SLEEP_MS / 1000) + "s -> sleeping stream");
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            try { nativeSetSleep(true); } catch (Throwable t) { /* native not up */ }
        }
    };

    /**
     * Android fallback for the in-app brightness slider, called from native only
     * when the Pico SDK backlight control didn't take. Sets this activity window's
     * brightness (0..1); needs no special permission. Must run on the UI thread.
     */
    public void setWindowBrightness(final float frac) {
        final float f = frac < 0f ? 0f : (frac > 1f ? 1f : frac);
        mMainHandler.post(new Runnable() {
            @Override public void run() {
                try {
                    WindowManager.LayoutParams lp = getWindow().getAttributes();
                    lp.screenBrightness = f;
                    getWindow().setAttributes(lp);
                    Log.i(TAG, "brightness: Android window fallback -> " + f);
                } catch (Throwable t) {
                    Log.w(TAG, "brightness: window fallback failed", t);
                }
            }
        });
    }

    // ---- adb test hook: fire in-app popups without the real trigger condition ----
    // Exported broadcast that arms the low-battery popup, for QA on device:
    //   adb shell am broadcast -a org.meumeu.wivrn.neo2.pvr.TEST_BATTERY_WARN --ei pct 15 -p org.meumeu.wivrn.neo2.pvr
    // Omit --ei pct to default to 15%; use --ei pct 5 for the critical (red) variant.
    static final String ACTION_TEST_BATTERY_WARN = "org.meumeu.wivrn.neo2.pvr.TEST_BATTERY_WARN";
    private final BroadcastReceiver mTestReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context ctx, Intent intent) {
            if (intent == null) return;
            if (ACTION_TEST_BATTERY_WARN.equals(intent.getAction())) {
                int pct = intent.getIntExtra("pct", 15);
                Log.i(TAG, "test intent -> low-battery popup at " + pct + "%");
                try { nativeTestBatteryWarn(pct); } catch (Throwable t) { Log.e(TAG, "nativeTestBatteryWarn failed", t); }
            }
        }
    };
    private void registerTestReceiver() {
        IntentFilter f = new IntentFilter(ACTION_TEST_BATTERY_WARN);
        try {
            // API 33+ requires the export flag; pre-33 dynamic receivers are exported
            // by default, which is what lets `adb shell am broadcast` (shell uid) reach it.
            if (android.os.Build.VERSION.SDK_INT >= 33) {
                registerReceiver(mTestReceiver, f, Context.RECEIVER_EXPORTED);
            } else {
                registerReceiver(mTestReceiver, f);
            }
            Log.i(TAG, "test-intent receiver registered (" + ACTION_TEST_BATTERY_WARN + ")");
        } catch (Throwable t) { Log.e(TAG, "registerTestReceiver failed", t); }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // Request dangerous permissions upfront. CAMERA drives passthrough,
        // RECORD_AUDIO drives mic-to-PC streaming; both surface the system
        // grant dialog here so toggling mic in settings doesn't silently fail.
        java.util.ArrayList<String> perms = new java.util.ArrayList<>();
        if (checkSelfPermission(android.Manifest.permission.CAMERA)
                != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            perms.add(android.Manifest.permission.CAMERA);
        }
        if (checkSelfPermission(android.Manifest.permission.RECORD_AUDIO)
                != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            perms.add(android.Manifest.permission.RECORD_AUDIO);
        }
        if (!perms.isEmpty()) {
            requestPermissions(perms.toArray(new String[0]), 1001);
        }

        acquireWifiLocks();

        extractControllerModels();

        mUnityPlayer = new UnityPlayer(this);
        setContentView(mUnityPlayer);
        mUnityPlayer.surfaceView.getHolder().addCallback(this);

        // Long-lived render thread; surface comes/goes via the callbacks below
        // (so sleep/wake = surface destroy/recreate is handled gracefully).
        nativeStart(this);

        serverDiscovery = new ServerDiscovery(this);
        serverDiscovery.startDiscovery();
        startServerSyncThread();

        // Create the 2D Canvas-based VR UI (WivrnLobbyView)
        mVrUiPanel = new VrUiPanel(this);
        mVrUiPanel.attach();

        setupControllers();
        setupProximitySleep();
        registerTestReceiver();
        handleWivrnIntent(getIntent());
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        handleWivrnIntent(intent);
    }

    private void handleWivrnIntent(Intent intent) {
        if (intent == null || intent.getData() == null) return;
        Uri uri = intent.getData();
        String scheme = uri.getScheme();
        if (scheme == null) return;
        if (!scheme.equals("wivrn") && !scheme.equals("wivrn+tcp") && !scheme.equals("wivrn+udp")) return;

        String host = uri.getHost();
        int port = uri.getPort();
        if (port <= 0) port = 9757;
        boolean tcpOnly = scheme.equals("wivrn+tcp");
        String tcpOnlyParam = uri.getQueryParameter("tcp_only");
        if (tcpOnlyParam != null && !tcpOnlyParam.isEmpty()) {
            tcpOnly = tcpOnlyParam.equals("1") || tcpOnlyParam.equalsIgnoreCase("true");
        }

        String pin = null;
        String userInfo = uri.getUserInfo();
        if (userInfo != null) {
            int colon = userInfo.indexOf(':');
            if (colon >= 0 && colon + 1 < userInfo.length()) {
                pin = userInfo.substring(colon + 1);
            }
        }
        if (pin == null || pin.isEmpty()) {
            String query = uri.getQueryParameter("pin");
            if (query != null && !query.isEmpty()) pin = query;
        }

        Log.i(TAG, "wivrn intent: host=" + host + " port=" + port + " tcp=" + tcpOnly + " pin=" + (pin != null ? "yes" : "no"));

        if (host != null && !host.isEmpty()) {
            boolean sameTarget = host.equals(pendingHost) && port == pendingPort && tcpOnly == pendingTcpOnly;
            long now = System.currentTimeMillis();
            if (nativeConnecting && sameTarget) {
                Log.i(TAG, "intent: already connecting to " + host + ":" + port + ", ignoring duplicate");
                return;
            }
            if (sameTarget && hasPendingConnection && (now - lastConnectFlushMs) < CONNECT_DEBOUNCE_MS) {
                if (pin != null && !pin.isEmpty()) {
                    Log.i(TAG, "intent debounce: updating pin only for " + host + ":" + port);
                    pendingPin = pin;
                    nativeSetPin(pin);
                } else {
                    Log.i(TAG, "intent debounce: ignoring duplicate for " + host + ":" + port);
                }
                return;
            }
            pendingHost = host;
            pendingPort = port;
            pendingTcpOnly = tcpOnly;
            pendingPin = pin;
            hasPendingConnection = true;
            if (serverDiscovery != null) {
                serverDiscovery.addOrUpdateServer(host, host, port, tcpOnly);
            }
            flushPendingConnection();
        }
    }

    private void flushPendingConnection() {
        if (!hasPendingConnection) return;
        if (!nativeReady()) {
            Log.d(TAG, "native not ready, waiting to flush pending connection");
            new Thread(() -> {
                for (int i = 0; i < 50 && !nativeReady(); i++) {
                    try { Thread.sleep(100); } catch (InterruptedException e) { return; }
                }
                runOnUiThread(() -> flushPendingConnection());
            }, "pending-conn").start();
            return;
        }
        Log.i(TAG, "flushing pending connection: " + pendingHost + ":" + pendingPort + " tcp=" + pendingTcpOnly);
        lastConnectFlushMs = System.currentTimeMillis();
        if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null) {
            try { nativeSetBitrate(mVrUiPanel.getLobbyView().getBitrate()); } catch (Throwable t) {}
            try { nativeSetFoveationEnabled(mVrUiPanel.getLobbyView().getFoveationEnabled()); } catch (Throwable t) {}
            mVrUiPanel.getLobbyView().markAutoconnectAttempted();
        }
        nativeConnect(pendingHost, pendingPort, pendingTcpOnly);
        nativeConnecting = true;
        if (pendingPin != null && !pendingPin.isEmpty()) {
            nativeSetPin(pendingPin);
        }
        hasPendingConnection = false;
    }

    @Override
    protected void onResume() {
        super.onResume();
        mForeground = true;
        if (serverDiscovery != null) serverDiscovery.startDiscovery();
        flushPendingConnection();
        if (!hasPendingConnection && serverDiscovery != null) {
            serverDiscovery.tryAutoconnect(this);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        mForeground = false;
        if (serverDiscovery != null) serverDiscovery.stopDiscovery();
    }

    // Listen to the headset's proximity sensor for don/doff. "far" starts the
    // off-head sleep timer; "near" cancels it and wakes the stream back up.
    private void setupProximitySleep() {
        try {
            mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
            if (mSensorManager == null) { Log.e(TAG, "no SensorManager"); return; }
            mProximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
            if (mProximity == null) { Log.e(TAG, "no proximity sensor"); return; }
            mSensorManager.registerListener(mProximityListener, mProximity,
                    SensorManager.SENSOR_DELAY_NORMAL);
            Log.i(TAG, "proximity sensor registered (maxRange=" + mProximity.getMaximumRange() + ")");
        } catch (Throwable t) {
            Log.e(TAG, "setupProximitySleep failed", t);
        }
    }

    private final SensorEventListener mProximityListener = new SensorEventListener() {
        @Override public void onSensorChanged(SensorEvent event) {
            if (event.values.length == 0) return;
            // Convention: distance < maxRange => something is near (headset on head).
            boolean near = event.values[0] < mProximity.getMaximumRange();
            if (near) {
                // Donned: cancel any pending sleep and wake if we slept.
                mMainHandler.removeCallbacks(mSleepRunnable);
                if (mAsleep) {
                    mAsleep = false;
                    Log.i(TAG, "proximity: donned -> waking stream");
                    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                    try { nativeSetSleep(false); } catch (Throwable t) { /* native not up */ }
                }
            } else {
                // Doffed: arm the sleep timer (no-op if already armed/asleep).
                if (!mAsleep) {
                    mMainHandler.removeCallbacks(mSleepRunnable);
                    mMainHandler.postDelayed(mSleepRunnable, PROX_SLEEP_MS);
                }
            }
        }
        @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    };

    // Bind the Pico CV controller service. On success, start the SDK controller
    // thread (6DoF) and our poll loop. Poll state is pushed to native each tick.
    private void setupControllers() {
        try {
            ControllerClient.registerBindCallback(new BindControllerCallback() {
                @Override public void bindSuccess() {
                    // CRITICAL handedness fix: tell the CV service our "unity version".
                    // The service's getPvrMainController() branches on callunityversion:
                    // when it's false (we never set it) AND not-all-controllers-connected,
                    // it FORCES the main controller to index 1 and collapses both virtual
                    // hands onto one controller, this is the cold-start handedness swap.
                    // Setting a version flips callunityversion=true so the service uses the
                    // real per-controller state to assign handedness. (This is exactly what
                    // the system's CVControllerManager does on a home/back, which is why
                    // that "fixes" it.) Must happen before startControllerThread.
                    try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) {
                        Log.e(TAG, "setUnityVersion failed", t);
                    }
                    // Do NOT start the 6DoF thread here, wait until both controllers
                    // are connected so handedness binds correctly (see field comment).
                    Log.i(TAG, "controller service bound (unity version set); deferring 6DoF start until both connected");
                    startControllerPoll();
                }
                @Override public void unbindSuccess() {
                    Log.i(TAG, "controller service unbound");
                    stopControllerPoll();
                }
                @Override public void controllerConnectStateChanged(int hand, int state) {
                    Log.i(TAG, "controller " + hand + " connect state -> " + state);
                    // Just record state; the poll loop enforces the settle window
                    // before the one-shot start (see maybeStartControllerThread).
                    if (hand == 0) mConn0 = (state == 1);
                    else if (hand == 1) mConn1 = (state == 1);
                }
            });
            ControllerClient.bindControllerService(this);
            Log.i(TAG, "controller service bind requested");
        } catch (Throwable t) {
            Log.e(TAG, "setupControllers failed", t);
        }
    }

    // Start the 6DoF controller thread exactly once, only after both controllers
    // have been continuously connected for CTRL_SETTLE_MS. The service reports
    // "connected" instantly on bind (cached), but the link isn't truly
    // re-established yet; binding handedness that early causes the swap. Waiting
    // for it to hold steady lets the service settle so the bind is correct.
    // With only one controller, we rely on the VR Shell's pre-existing thread.
    private synchronized void maybeStartControllerThread() {
        if (mCtrlThreadStarted) return;
        if (!(mConn0 && mConn1)) {
            mBothConnSinceMs = 0;   // link dropped -> restart the settle window
            return;
        }
        long now = android.os.SystemClock.uptimeMillis();
        if (mBothConnSinceMs == 0) {
            mBothConnSinceMs = now;
            Log.i(TAG, "both controllers connected -> settling " + mSettleMs + "ms before 6DoF start");
            return;
        }
        if (now - mBothConnSinceMs < mSettleMs) return;
        try {
            try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) { /* logged at bind */ }
            ControllerClient.startControllerThread(1, 1);
            mCtrlThreadStarted = true;
            Log.i(TAG, "controller link settled -> startControllerThread (handedness bound)");
        } catch (Throwable t) {
            Log.e(TAG, "startControllerThread failed", t);
        }
    }

    // Reclaim the SHARED CV service after a full-screen takeover stole it (see
    // mSurfaceWasDestroyed). Replicates the proven Home/back recovery without a full
    // rebind: re-assert OUR unity version (overriding the shortcut's 2.9.0.0) and let
    // the (short) settle path re-issue startControllerThread, which re-anchors the
    // controller link to our session's head frame. No-op if controllers never started.
    private synchronized void reclaimControllerLink() {
        if (!mCtrlRunning) return;   // service not bound/polling yet -> nothing to reclaim
        try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) {
            Log.e(TAG, "reclaim setUnityVersion failed", t);
        }
        mSettleMs = CTRL_RECLAIM_SETTLE_MS;   // both hands already healthy -> short window
        mBothConnSinceMs = 0;                 // restart the settle clock
        mCtrlThreadStarted = false;           // allow the poll loop to re-issue the start
        Log.i(TAG, "surface returned after takeover -> reclaiming CV controller link");
    }

    private void startControllerPoll() {
        if (mCtrlRunning) return;
        mCtrlRunning = true;
        mCtrlThread = new Thread(() -> {
            while (mCtrlRunning) {
                // Fallback for the deferred first start: if the connect-state callback
                // didn't fire (or fired before the poll thread existed), pick up the
                // live connection state here and start once both are connected.
                if (!mCtrlThreadStarted) {
                    try {
                        mConn0 = (ControllerClient.getControllerConnectionState(0) == 1);
                        mConn1 = (ControllerClient.getControllerConnectionState(1) == 1);
                        maybeStartControllerThread();
                    } catch (Throwable t) { /* service not ready; retry next tick */ }
                }
                for (int h = 0; h < 2; h++) {
                    try {
                        int conn = ControllerClient.getControllerConnectionState(h);
                        // Occasional diagnostic: log sensor status + 6DoF pose every
                        // ~30s to help diagnose tracking issues if they recur.
                        if (conn == 1 && h == 0 && (diagTick++ % 2700) == 0) {
                            try {
                                int sstat = ControllerClient.getControllerSensorStatus(h);
                                float[] pose6dof = ControllerClient.getController6dofPose(h);
                                String p6s = pose6dof != null
                                    ? String.format("(%.1f,%.1f,%.1f)", pose6dof[4], pose6dof[5], pose6dof[6])
                                    : "null";
                                // Also log what each sensor state variant returns
                                float[] ss1 = null, ss2 = null;
                                try { ss1 = ControllerClient.getControllerSensorState(h, mHeadData); } catch (Throwable e) {}
                                try { ss2 = ControllerClient.getControllerSensorStateBySharmem(h, mHeadData); } catch (Throwable e) {}
                                String ss1s = ss1 != null
                                    ? String.format("(%.1f,%.1f,%.1f)", ss1[4], ss1[5], ss1[6]) : "null";
                                String ss2s = ss2 != null
                                    ? String.format("(%.1f,%.1f,%.1f)", ss2[4], ss2[5], ss2[6]) : "null";
                                Log.i(TAG, "DIAG h=" + h + " conn=" + conn
                                    + " sensorStatus=" + sstat
                                    + " 6dofPose=" + p6s
                                    + " sensorState=" + ss1s
                                    + " sharmem=" + ss2s
                                    + " head=(" + mHeadData[4] + "," + mHeadData[5] + "," + mHeadData[6] + ")");
                            } catch (Throwable dt) { /* diagnostic only */ }
                        }
                        // Head-aligned pose: feed the live head pose so the CV service
                        // returns the controller in the HEAD's tracking frame (this is
                        // what the legacy SDK does when enablehand6dofbyhead==1). Keeps
                        // controllers locked to the head instead of a separate universe.
                        nativeGetHeadData(mHeadData);
                        // Try the head-data + pre-time variant first: it uses a
                        // different code path that may work when the plain
                        // getControllerSensorState(h, head) returns zero position
                        // (broken by the VR Shell's stale CV service state).
                        float[] sensor = null;
                        try {
                            ControllerClient.SetHeadDataAndPreTime(mHeadData, 0.0f);
                            sensor = ControllerClient.getControllerSensorStateWithHeadDataAndPreTime(h);
                        } catch (Throwable t) { /* fall through to next variant */ }
                        if (sensor == null || (sensor.length >= 7
                                && sensor[4] == 0.0f && sensor[5] == 0.0f && sensor[6] == 0.0f)) {
                            try {
                                sensor = ControllerClient.getControllerSensorStateBySharmem(h, mHeadData);
                            } catch (Throwable t) { /* fall through */ }
                        }
                        if (sensor == null || (sensor.length >= 7
                                && sensor[4] == 0.0f && sensor[5] == 0.0f && sensor[6] == 0.0f)) {
                            try { sensor = ControllerClient.getControllerSensorState(h, mHeadData); }
                            catch (Throwable t) { /* fall through */ }
                        }
                        // Fallback: when the CV service's head-aligned transform is
                        // broken (e.g. the VR Shell left it in a bad state for the
                        // one-controller case), getControllerSensorState returns a zero
                        // pose even though the controller IS tracking. Fall back to the
                        // raw 6DoF pose which bypasses the head-frame transform.
                        if (sensor != null && sensor.length >= 7
                                && sensor[4] == 0.0f && sensor[5] == 0.0f && sensor[6] == 0.0f) {
                            try {
                                float[] pose6 = ControllerClient.getController6dofPose(h);
                                if (pose6 != null && pose6.length >= 7
                                        && (pose6[4] != 0.0f || pose6[5] != 0.0f || pose6[6] != 0.0f)) {
                                    sensor = pose6;
                                }
                            } catch (Throwable ft) { /* 6dof pose not available */ }
                        }
                        float[] angVel = ControllerClient.getControllerAngularVelocity(h);
                        // Button/joystick state lives in getControllerKeyEventUnityExt
                        // (a ~67-int array). Reverse-engineered held-state indices on
                        // this CV2 (FalconCV2) controller (see EXT_* below). We pack the
                        // values we need into a compact, fixed-layout array for native:
                        //   keys[0]=joyX  keys[1]=joyY (analog, center 128)
                        //   keys[2]=trigger  keys[3]=grip  keys[4]=joyClick  keys[5]=menu
                        int[] ext = ControllerClient.getControllerKeyEventUnityExt(h);
                        int[] legacy = ControllerClient.getControllerKeyEvent(h);
                        int gripAnalog = 0;
                        try { gripAnalog = ControllerClient.getControllerGripValue(h); } catch (Throwable t) {}
                        int[] keys = new int[11];
                        keys[0] = pick(ext, EXT_JOY_X);      // joystick X, center 128
                        keys[1] = pick(ext, EXT_JOY_Y);      // joystick Y, center 128
                        keys[2] = pick(ext, EXT_TRIGGER);    // trigger digital (0/1)
                        keys[3] = pick(ext, h == 1 ? EXT_GRIP_RIGHT : EXT_GRIP_LEFT); // grip (0/1), per-hand index
                        keys[4] = pick(ext, EXT_JOY_CLICK);  // joystick click (0/1)
                        keys[5] = pick(ext, EXT_MENU);       // app/menu (0/1)
                        keys[6] = pick(ext, EXT_AX);         // A (right) / X (left) (0/1)
                        keys[7] = pick(ext, EXT_BY);         // B (right) / Y (left) (0/1)
                        keys[8] = pick(legacy, LEGACY_TRIG); // trigger analog 0..255
                        keys[9] = gripAnalog;                // grip analog 0..255
                        keys[10] = pick(ext, EXT_BATTERY);   // controller battery (0..100, diag HUD)
                        // Backgrounded (Home / app not in foreground): report the
                        // controller as disconnected so the native uplink drops it
                        // (the render thread skips conn != 1), no controller tracking
                        // or button input reaches the server while the user is away.
                        int sendConn = mForeground ? conn : 0;
                        nativeControllerState(h, sendConn, sensor, angVel, keys);
                        // Always drain the parked haptic so a stale pulse can't buzz on
                        // resume, but only actually rumble the CV2 wand while foreground
                        // and connected. strength 0..1, time ms, hand.
                        boolean haveHaptic = nativeDrainHaptic(h, mHaptic);
                        if (mForeground && conn == 1 && haveHaptic) {
                            try {
                                ControllerClient.vibrateCV2ControllerStrength(
                                        mHaptic[0], (int) mHaptic[1], h);
                            } catch (Throwable t) { /* old service: no CV2 vibrate */ }
                        }
                    } catch (Throwable t) {
                        // service not ready yet / transient; keep polling
                    }
                }
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
            }
        }, "ctrl-poll");
        mCtrlThread.start();
        Log.i(TAG, "controller poll thread started");
    }

    // getControllerKeyEventUnityExt index layout, from the Pico legacy SDK source
    // (Pvr_ControllerManager.TransformData, per-hand offset 0):
    //   0=joyX 5=joyY 10=Home 15=App(menu) 20=touch 35=Trigger 40=Battery
    //   45=A/X 50=B/Y 55=Right-grip(R hand) 60=Left-grip(L hand) 66=touchpad Click
    private static final int EXT_JOY_X      = 0;
    private static final int EXT_JOY_Y      = 5;
    private static final int EXT_MENU       = 15;  // App button
    private static final int EXT_TRIGGER    = 35;
    private static final int EXT_BATTERY    = 40;  // controller battery level (0..100)
    private static final int EXT_AX         = 45;  // A (right) / X (left)
    private static final int EXT_BY         = 50;  // B (right) / Y (left)
    private static final int EXT_GRIP_RIGHT = 55;  // grip on the right controller
    private static final int EXT_GRIP_LEFT  = 60;  // grip on the left controller
    // Thumbstick CLICK. The SDK exposes TWO clicks depending on mode: in the lobby's
    // touchpad/pointer mode it's TouchPadClick=ext[66]; in 6DoF/joystick mode (i.e.
    // while STREAMING, the case that matters) ext[66] is dead and the stick press
    // is the "Touch" key State at ext[20] (held while pressed). Use 20 so click works
    // in-stream. (Verified on-device: streaming clicks toggle ext[20], never ext[66].)
    private static final int EXT_JOY_CLICK  = 20;
    private static final int LEGACY_TRIG    = 7;   // analog trigger 0..255 in getControllerKeyEvent
    private static int pick(int[] a, int i) { return (a != null && i < a.length) ? a[i] : 0; }

    private void stopControllerPoll() {
        mCtrlRunning = false;
        if (mCtrlThread != null) { mCtrlThread.interrupt(); mCtrlThread = null; }
    }

    private void acquireWifiLocks() {
        try {
            WifiManager wifi = (WifiManager)
                    getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            if (wifi == null) { Log.e(TAG, "no WifiManager"); return; }
            mMulticastLock = wifi.createMulticastLock("wivrn-mdns");
            mMulticastLock.setReferenceCounted(false);
            mMulticastLock.acquire();
            mWifiLock = wifi.createWifiLock(
                    WifiManager.WIFI_MODE_FULL_HIGH_PERF, "wivrn-stream");
            mWifiLock.setReferenceCounted(false);
            mWifiLock.acquire();
            Log.i(TAG, "wifi locks acquired: multicast=" + mMulticastLock.isHeld()
                    + " wifi=" + mWifiLock.isHeld());
        } catch (Throwable t) {
            Log.e(TAG, "acquireWifiLocks failed", t);
        }
    }

    private void releaseWifiLocks() {
        try {
            if (mMulticastLock != null && mMulticastLock.isHeld()) mMulticastLock.release();
            if (mWifiLock != null && mWifiLock.isHeld()) mWifiLock.release();
        } catch (Throwable t) { Log.e(TAG, "releaseWifiLocks failed", t); }
    }

    // Forward headset/controller key presses to native. The side OK button drives
    // the lobby Software-IPD slider. We forward all keycodes (native logs them so
    // we can identify the OK button), and consume the OK candidates so the system
    // doesn't also act on them; everything else passes through normally.
    // The Pico side OK button is custom keycode 1001 (no KeyEvent constant).
    private static final int KEYCODE_PICO_OK = 1001;
    private static boolean isOkKey(int keyCode) {
        return keyCode == KEYCODE_PICO_OK
                || keyCode == KeyEvent.KEYCODE_ENTER
                || keyCode == KeyEvent.KEYCODE_DPAD_CENTER
                || keyCode == KeyEvent.KEYCODE_BUTTON_A;
    }

    // Called from native to open a URL in the system browser.
    public void openUrl(String url) {
        try {
            Intent intent = new Intent(Intent.ACTION_VIEW, Uri.parse(url));
            startActivity(intent);
        } catch (Exception e) {
            Log.e(TAG, "Failed to open URL: " + url, e);
        }
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        nativeKeyEvent(keyCode, true);
        if (isOkKey(keyCode)) return true;
        return super.onKeyDown(keyCode, event);
    }

    @Override
    public boolean onKeyUp(int keyCode, KeyEvent event) {
        nativeKeyEvent(keyCode, false);
        if (isOkKey(keyCode)) return true;
        return super.onKeyUp(keyCode, event);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        Log.i(TAG, "surfaceCreated");
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.i(TAG, "surfaceChanged " + width + "x" + height);
        if (width > 0 && height > 0) {
            nativeSurfaceChanged(holder.getSurface());
            // If we're coming back from a full-screen takeover (surface was destroyed),
            // the shared CV service was reconfigured by that app -> reclaim it.
            if (mSurfaceWasDestroyed) {
                mSurfaceWasDestroyed = false;
                reclaimControllerLink();
            }
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed");
        mSurfaceWasDestroyed = true;   // arm CV-service reclaim for when the surface returns
        nativeSurfaceDestroyed();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try { unregisterReceiver(mTestReceiver); } catch (Throwable t) { /* not registered */ }
        mMainHandler.removeCallbacks(mSleepRunnable);
        if (mSensorManager != null) {
            try { mSensorManager.unregisterListener(mProximityListener); } catch (Throwable t) { /* ignore */ }
        }
        stopControllerPoll();
        stopServerSyncThread();
        if (serverDiscovery != null) serverDiscovery.stopDiscovery();
        try { ControllerClient.unbindControllerService(this); } catch (Throwable t) { /* ignore */ }
        nativeStop();
        releaseWifiLocks();
    }

    private void startServerSyncThread() {
        if (mServerSyncRunning) return;
        mServerSyncRunning = true;
        mServerSyncThread = new Thread(() -> {
            int syncCount = 0;
            while (mServerSyncRunning) {
                try {
                    List<ServerDiscovery.ServerEntry> all = serverDiscovery.getAllServers();
                    String[] names = new String[all.size()];
                    String[] hosts = new String[all.size()];
                    int[] ports = new int[all.size()];
                    boolean[] tcpOnly = new boolean[all.size()];
                    boolean[] discovered = new boolean[all.size()];
                    boolean[] autoconnect = new boolean[all.size()];
                    for (int i = 0; i < all.size(); i++) {
                        ServerDiscovery.ServerEntry s = all.get(i);
                        names[i] = s.name;
                        hosts[i] = s.hostname;
                        ports[i] = s.port;
                        tcpOnly[i] = s.tcpOnly;
                        discovered[i] = s.discovered;
                        autoconnect[i] = s.autoconnect;
                    }
                    nativeSetServerList(names, hosts, ports, tcpOnly, discovered, autoconnect);
                    if (++syncCount % 60 == 0)
                        Log.i(TAG, "server list synced " + syncCount + " (" + all.size() + " servers)");
                } catch (Throwable t) {
                    Log.e(TAG, "server list sync error", t);
                }
                try { Thread.sleep(500); } catch (InterruptedException e) { break; }
            }
        }, "server-sync");
        mServerSyncThread.start();
        Log.i(TAG, "server list sync thread started");
    }

    private void stopServerSyncThread() {
        mServerSyncRunning = false;
        if (mServerSyncThread != null) { mServerSyncThread.interrupt(); mServerSyncThread = null; }
    }

    // Called from C++ (jni.cpp) when the user clicks CONNECT in the native 3D UI.
    public void onServerConnect(String hostname, int port, boolean tcpOnly) {
        if (nativeConnecting) {
            Log.i(TAG, "connect requested " + hostname + ":" + port + " tcp=" + tcpOnly + " ignored, already connecting");
            return;
        }
        Log.i(TAG, "connect requested " + hostname + ":" + port + " tcp=" + tcpOnly);
        try { nativeConnect(hostname, port, tcpOnly); } catch (Throwable t) { Log.e(TAG, "nativeConnect failed", t); }
    }

    // Called from C++ when the user clicks the X button on a server entry.
    public void onServerRemove(String hostname, int port) {
        Log.i(TAG, "remove server " + hostname + ":" + port);
        serverDiscovery.removeServer(hostname, port);
    }

    // Called from C++ when the user toggles the autoconnect switch.
    public void onServerAutoconnect(String hostname, int port) {
        Log.i(TAG, "toggle autoconnect " + hostname + ":" + port);
        serverDiscovery.setAutoconnect(hostname, port);
    }

    // Called from C++ when the user clicks the Refresh button.
    public void onRefreshServers() {
        Log.i(TAG, "refresh servers requested");
        serverDiscovery.refreshDiscovery();
    }

    // Called from C++ when the connecting state changes.
    public void onConnectingChanged(boolean connecting) {
        nativeConnecting = connecting;
        Log.i(TAG, "nativeConnecting = " + connecting);
    }

    // Called from C++ (streaming_client.cpp) when the server requests a PIN.
    public void requestPinEntry() {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().setConnectionState(WivrnLobbyView.STATE_PIN_ENTRY,
                        getString(org.meumeu.wivrn.neo2.pvr.R.string.enter_pin));
        });
    }

    // ---- WivrnLobbyView callbacks ----
    public void onIpdChanged(float ipdMm) {
        try { nativeSetIpd(ipdMm); } catch (Throwable t) { Log.e(TAG, "nativeSetIpd failed", t); }
    }
    public void onMicrophoneChanged(boolean enabled) {
        try { nativeSetMicrophone(enabled); } catch (Throwable t) { Log.e(TAG, "nativeSetMicrophone failed", t); }
    }
    public void onPinCancelled() {}
    public void onPinEntered(String pin) {
        try { nativeSetPin(pin); } catch (Throwable t) { Log.e(TAG, "nativeSetPin failed", t); }
    }
    public void onDisconnectRequested() {
        nativeConnecting = false;
        try { nativeDisconnect(); } catch (Throwable t) { Log.e(TAG, "nativeDisconnect failed", t); }
    }
    public void onReconnectRequested() {}
    public void onRenderResolutionChanged(int width, int height) {
        try { nativeSetRenderResolution(width, height); } catch (Throwable t) { Log.e(TAG, "nativeSetRenderResolution failed", t); }
    }
    public void onStreamResolutionChanged(int width, int height) {
        try { nativeSetStreamResolution(width, height); } catch (Throwable t) { Log.e(TAG, "nativeSetStreamResolution failed", t); }
    }
    public void onRequestAppList() {
        try { nativeRequestAppList(); } catch (Throwable t) { Log.e(TAG, "nativeRequestAppList failed", t); }
    }
    public void onRequestRunningApps() {
        try { nativeRequestRunningApps(); } catch (Throwable t) { Log.e(TAG, "nativeRequestRunningApps failed", t); }
    }
    public void onSetActiveApp(int appId) {
        try { nativeSetActiveApp(appId); } catch (Throwable t) { Log.e(TAG, "nativeSetActiveApp failed", t); }
    }
    public void onStartApp(String appId) {
        try { nativeStartApp(appId); } catch (Throwable t) { Log.e(TAG, "nativeStartApp failed", t); }
    }
    public void onStopApp(int appId) {
        try { nativeStopApp(appId); } catch (Throwable t) { Log.e(TAG, "nativeStopApp failed", t); }
    }

    // ---- streaming_client callbacks ----
    public void onConnectionStateChanged(int state, String message) {
        if (state == WivrnLobbyView.STATE_DISCONNECTED || state == WivrnLobbyView.STATE_IDLE) {
            nativeConnecting = false;
        }
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().setConnectionState(state, message);
        });
    }
    public void onStreamStats(int fps, int latencyMs, int bandwidthRx, int bandwidthTx, int bitrateMbps) {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().updateStreamStats(fps, latencyMs, bandwidthRx, bandwidthTx, bitrateMbps);
        });
    }
    public void onStreamStatsDetailed(float[] data) {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().updateStreamStatsDetailed(data);
        });
    }
    public void onApplicationList(String[] ids, String[] names) {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().updateAvailableApps(ids, names);
        });
    }
    public void onApplicationIcon(String appId, byte[] pngData) {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().updateAppIcon(appId, pngData);
        });
    }
    public void onRunningApplications(String[] names, int[] ids, boolean[] overlays, boolean[] actives) {
        runOnUiThread(() -> {
            if (mVrUiPanel != null && mVrUiPanel.getLobbyView() != null)
                mVrUiPanel.getLobbyView().updateRunningApps(names, ids, overlays, actives);
        });
    }
    public void onLobbyTouch(float x, float y, boolean down, boolean pressed, float thumbstickY) {
        if (mVrUiPanel != null)
            mVrUiPanel.setTouchState(x, y, down, pressed, thumbstickY);
    }
}
