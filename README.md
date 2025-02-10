
# M5Stack AtomS3 WiFi connection first!

This project enables your M5Stack AtomS3 sensor to connect to your local 2.4 Ghz WiFi network without needing to reflash the firmware. It uses a “provisioning mode” that lets you enter your WiFi credentials using a simple web page.

## How to connect it to your Wifi

When the sensor powers on, it checks if WiFi credentials have already been saved. If no credentials are found—or if you force configuration by pressing the built‑in button—it enters **Configuration Mode**. In this mode, the sensor:

1. Creates its own WiFi network (Access Point) with the following default settings:  
   - **SSID:** `M5Atoms3-Config`  
   - **Password:** `provision`
2. Displays a short set of instructions on the built‑in screen (using small text) that includes the SSID, password, and the URL to open.
3. Starts a simple web server that serves a page where you can enter your local WiFi credentials.
4. Saves the new credentials and restarts. On reboot, the sensor will attempt to connect to your WiFi network using the provided details.

## Step-by-Step Instructions for Non-Experts

1. **Power On the Sensor:**  
   Simply plug in your M5Stack AtomS3 sensor.  
   - If no WiFi credentials have been saved, the sensor automatically enters configuration mode.
   - Alternatively, press the built‑in button at startup to force configuration mode.

2. **Check the Built-in Display:**  
   The screen will clear and show a concise message (using small text) that looks similar to this:

   ```
   CONFIG MODE
   SSID: M5Atoms3-Config
   PW: provision
   URL: http://192.168.4.1
   ```

   *(Note: The URL may vary depending on your sensor’s AP IP address.)*

3. **Connect Your Phone or Computer:**  
   Open your WiFi settings and look for the network named **M5Atoms3-Config**.  
   - Connect using the password **provision**.

4. **Open the Provisioning Web Page:**  
   Once connected to the sensor’s WiFi network, open your web browser and navigate to the URL shown on the sensor’s display (for example, `http://192.168.4.1`).

5. **Enter Your WiFi Credentials:**  
   You will see a simple web page with a form. Enter the SSID and password of your home (or office) WiFi network and submit the form.

6. **Sensor Saves the Credentials:**  
   The sensor will store your WiFi credentials and then reboot.  
   After the restart, it will try to connect to your local WiFi network.

7. **Normal Operation:**  
   Once connected, the sensor will stop hosting its own AP and will operate normally as part of your local network.

## Troubleshooting Tips

- **No Connection?**  
  If the sensor fails to connect to your WiFi network, try forcing configuration mode by holding the built‑in button during power-up and re-enter your credentials.

- **Display Clutter:**  
  In configuration mode, only a few key lines are shown on the screen. All detailed logs are sent to the Serial Monitor for debugging.

- **Reconfiguring:**  
  You can reconfigure the sensor at any time by starting it in configuration mode (by pressing the built‑in button at startup).

## Dependencies

- [M5Atom Library](https://github.com/m5stack/M5Atom)  
- WiFi and WebServer libraries (included in the Arduino core for ESP32)  
- Preferences library for storing data in non‑volatile memory

## License

This project is provided under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Below is a **high‐level user guide** to the **Heave + Wave + Collision** measurement system, describing **what** the program does, **how** it presents results, and **which NMEA outputs** it sends. It's written for **end users** (e.g., boat operators or marine researchers) rather than developers diving into code details.

---

## 1. Overview of the System

This software runs on an **M5Atom S3** device (or similar M5 hardware with an IMU) to measure:

1. **Wave Frequency & Height**: How often waves occur (their period) and how large they are (average height, significant height).  
2. **Boat Motion** (Heave, Attitude):  
   - **Heave** (vertical displacement) is derived from the IMU’s acceleration, filtering out slow drifts and high‐frequency noise.  
   - **Attitude** (pitch, roll, and yaw angles) is tracked to see how the boat is oriented in real time.  
3. **Slamming / Jerk**: Quick changes in acceleration that indicate abrupt impacts with waves.  
4. **Collision‐Like Parameters** (peak accel & jerk in last second): Another system can use them to detect dangerous impacts.

**Optionally**, the system can operate in **simulation mode**, generating a sinusoidal wave acceleration internally for testing or demonstration purposes.

---

## 2. How It Works at a High Level

1. **Reads IMU or Generates Data**:  
   - **Real Mode**: Reads the built‐in IMU’s acceleration, removing gravity via an advanced orientation filter (Madgwick).  
   - **Simulation Mode**: Calculates a synthetic wave of chosen period and height, emulating boat motion on a sinusoidal wave.

2. **Filters & Integrates**:  
   - A **band‐pass filter** keeps only the frequency range relevant to waves (e.g., 0.03–1 Hz).  
   - The filtered acceleration is **integrated** twice to estimate vertical displacement (heave).

3. **Analyzes** Wave & Boat Behavior:  
   - **Wave Frequency** (via zero‐cross counting in the acceleration).  
   - **Wave Height** (crest‐to‐trough in the integrated displacement), giving “average” and “significant” wave height.  
   - **Slamming** (sudden changes in acceleration = jerk).  
   - **Collision**: Peak acceleration & jerk in the last ~1 second (helpful for other systems to detect hull stress).  
   - **Attitude**: Captures pitch/roll/yaw amplitude and RMS (useful to see how the boat is handling waves).

4. **Presents** Results via:  
   - **On‐Device Display**: A compact text readout (wave frequency, wave height, jerk, collisions, pitch/roll/yaw, etc.).  
   - **NMEA 0183** “XDR” messages broadcast over UDP to the local network.

---

## 3. What the User Sees

### 3.1 M5Atom S3 Display

- A **short text** display with lines like:

  ```
  BP=0.03-1.00Hz
  F=0.10 P=10.00
  AH=2.00 SH=3.00
  Jrk=0.6/1.2
  Bft=3
  pA=5.0 rA=6.2
  yA=1.0
  pR=2.0 rR=2.5
  yR=1.0
  pN=2.5 rN=2.0
  yN=-1.2
  PkA=3.2
  PkJ=15.4
  ```

  - **BP=0.03–1.00Hz**: The band‐pass range.  
  - **F=0.10 P=10.00**: Wave frequency = 0.10 Hz, period = 10.0 s.  
  - **AH=2.00 SH=3.00**: Average wave height = 2 m, significant wave height = 3 m.  
  - **Jrk=0.6/1.2**: Instant jerk = 0.6 m/s³, integrated jerk = 1.2 m/s².  
  - **Bft=3**: Beaufort scale estimate.  
  - **pA=5.0 rA=6.2 yA=1.0**: Peak‐to‐peak amplitude of pitch, roll, yaw in degrees.  
  - **pR=2.0 rR=2.5 yR=1.0**: RMS for pitch, roll, yaw.  
  - **pN=2.5 rN=2.0 yN=-1.2**: Current pitch, roll, yaw angles.  
  - **PkA=3.2 PkJ=15.4**: Peak acceleration ~3.2 m/s² & peak jerk ~15.4 m/s³ in last second.

### 3.2 NMEA 0183 Output

The program **broadcasts** (UDP) a series of `$XDR` sentences once per second. Each line looks like:

```
$XDR,A,xx.xx,UNIT,LABEL*CHK
```
where:

- **LABEL** is one of:  
  - `Frequency`, `Period`, `AvgWaveH`, `SigWaveH`, `InstJerk`, `IntJerk`, `Beaufort`  
  - `PitchAmp`, `RollAmp`, `YawAmp`, `PitchRMS`, `RollRMS`, `YawRMS`, `PitchNow`, `RollNow`, `YawNow`  
  - `PeakAccel`, `PeakJerk`  
- **UNIT** matches the parameter type, e.g.:  
  - `HZ` for wave frequency  
  - `S` for period  
  - `M` for wave height  
  - `M/S2` for acceleration  
  - `M/S3` for jerk  
  - `D` (degrees) for angles  
- `xx.xx` is a floating‐point numeric value, followed by a `*` and a 2‐digit hexadecimal checksum.

For instance:

```
$XDR,A,0.10,HZ,Frequency*7C
$XDR,A,10.00,S,Period*21
$XDR,A,2.00,M,AvgWaveH*4F
$XDR,A,3.00,M,SigWaveH*5A
$XDR,A,0.60,M/S3,InstJerk*2A
$XDR,A,1.20,M/S2,IntJerk*01
$XDR,A,3.00,,Beaufort*7B
$XDR,A,5.00,D,PitchAmp*19
$XDR,A,2.00,D,PitchRMS*55
...
$XDR,A,3.20,M/S2,PeakAccel*37
$XDR,A,15.40,M/S3,PeakJerk*78
```

Any marine software listening on the local network can interpret these wave, collision, and attitude values.

---

## 4. Typical Usage

1. **Mount** the M5 device near your boat’s center of gravity or another stable point.  
2. **Power** it on and wait for Wi‐Fi.  
3. **Observe** wave frequency/period, wave heights on the M5’s small display.  
4. **Check** the attitude lines to see if pitch/roll remain within safe limits.  
5. If you want **simulation** for bench testing, set `simulationMode=true`. The code will produce a synthetic wave to confirm each part of the pipeline.  
6. **Collect** real data in your marine software by reading the `$XDR` messages broadcast over UDP. Parameters can be used for real‐time wave info or collision alarms.

---

## 5. Key Notes

- The **band‐pass filter** is crucial in real conditions to avoid huge offsets. You can tweak `LOW_CUTOFF_HZ` and `HIGH_CUTOFF_HZ` depending on your expected wave range.  
- The system is designed so that **real** and **simulation** modes both use the **same** filtering, integration, wave analysis, and **NMEA** outputs. This makes debugging consistent.  
- For **collision** monitoring, watch the “PeakAccel” and “PeakJerk” XDR messages. Another system can set thresholds to raise alarms if these exceed safe limits.  
- The **Beaufort** scale is just an approximation from wave height. Real conditions can vary widely.

---

### Summary

This wave measurement code provides a **complete** solution for:

- **Wave frequency** & **height** (avg & significant),
- **Slamming** / jerk stats,
- **Collision** detection parameters,
- **Boat attitude** amplitude & RMS,
- **NMEA** output for marine integration,
- **Small M5 display** for local readouts,
- **Optionally** simulating a known wave to test or demonstrate the system.  

All while removing drift via **band‐pass** filtering, ensuring stable wave amplitude measurement in real conditions.

Below is a **complete** reference describing **every** NMEA sentence the code sends, **all** labels, **units**, and **meanings**, plus the **general XDR format** used for each message. This guide ensures nothing is omitted, so you have a full specification of the system’s NMEA outputs.

---

## 1. General NMEA XDR Format

Each parameter is transmitted once per second in an **NMEA 0183** “XDR” sentence, which has the following structure:

```
$XDR,A,<value>,<unit>,<label>*<checksum>\r\n
```

1. **$XDR**: The NMEA sentence type (Transducer Measurements).  
2. **A**: Indicates the **transducer type** is “acceleration or generic,” as used in many XDR expansions.  
3. **<value>**: A floating‐point number representing the measured quantity (e.g., `1.23` or `10.0`).  
4. **<unit>**: The measurement unit (e.g. `HZ`, `S`, `M`, `M/S2`, `D` for degrees, etc.).  
5. **<label>**: A short string label that identifies which parameter this is (e.g. `Frequency`, `PitchRMS`, etc.).  
6. ***<checksum>**: A 2‐digit hexadecimal checksum computed by XORing all characters between `$` and `*`.

Example:
```
$XDR,A,0.10,HZ,Frequency*7C\r\n
```
Here, `0.10` is the value (wave frequency in Hz), `HZ` is the unit, `Frequency` is the label, and `7C` is the checksum.

---

## 2. Full List of Output Sentences

Below is a **complete** set of XDR messages sent by the code. Each line’s `<label>` matches the code’s `sendNmeaXDR(...)` call, and `<unit>` corresponds to the parameter’s dimension.

### 2.1 Wave Frequency & Period

1. **Frequency**  
   - **Format**: `XDR,A,<value>,HZ,Frequency`  
   - **Meaning**: The dominant wave frequency in **Hertz** (cycles per second), derived by zero‐cross counting on filtered acceleration.  
   - **Typical Range**: ~0.03–1.0 Hz for ocean waves, depending on wave periods from ~1 s to ~30 s.

2. **Period**  
   - **Format**: `XDR,A,<value>,S,Period`  
   - **Meaning**: The wave’s fundamental period in **seconds** (1 / frequency).  
   - **Typical Range**: ~1–30 s for typical seas.

### 2.2 Wave Heights

3. **AvgWaveH**  
   - **Format**: `XDR,A,<value>,M,AvgWaveH`  
   - **Meaning**: The **average** crest‐to‐trough wave height in **meters**, computed over the last ring buffer.  
   - **Typical Range**: < 0.1 m in calm seas up to several meters in higher seas.

4. **SigWaveH**  
   - **Format**: `XDR,A,<value>,M,SigWaveH`  
   - **Meaning**: The **significant** wave height in **meters** (average height of top 1/3 largest waves). Often considered a key oceanographic measure of sea state.

### 2.3 Slamming / Jerk

5. **InstJerk**  
   - **Format**: `XDR,A,<value>,M/S3,InstJerk`  
   - **Meaning**: The **instantaneous jerk** (rate of acceleration change) at the latest sample, in **m/s³**.  
   - **Typical Range**: 0–many tens m/s³ if the boat slams abruptly.

6. **IntJerk**  
   - **Format**: `XDR,A,<value>,M/S2,IntJerk`  
   - **Meaning**: The integrated absolute jerk over the entire buffer, in **m/s²** (dimension is effectively jerk × time). Helpful as a “slamming severity” measure.

### 2.4 Beaufort

7. **Beaufort**  
   - **Format**: `XDR,A,<value>,,Beaufort`  
   - **Meaning**: An integer (0–12 or more) representing an approximate **Beaufort scale** level, derived from wave height.  
   - **Typical Range**: 0 = calm, 12 = hurricane‐force conditions.  
   - **Unit**: none, so `<unit>` is empty.

### 2.5 Attitude Motion

8. **PitchAmp**  
   - **Format**: `XDR,A,<value>,D,PitchAmp`  
   - **Meaning**: Pitch **peak‐to‐peak amplitude** in **degrees** over the ring buffer.  
   - **Typical Range**: 0–30° or more in rough seas.

9. **RollAmp**  
   - **Format**: `XDR,A,<value>,D,RollAmp`  
   - **Meaning**: Roll **peak‐to‐peak amplitude** in degrees.  

10. **YawAmp**  
    - **Format**: `XDR,A,<value>,D,YawAmp`  
    - **Meaning**: Yaw **peak‐to‐peak amplitude** in degrees.

11. **PitchRMS**  
    - **Format**: `XDR,A,<value>,D,PitchRMS`  
    - **Meaning**: **Root‐Mean‐Square** pitch angle in degrees, measuring average pitch motion.

12. **RollRMS**  
    - **Format**: `XDR,A,<value>,D,RollRMS`  
    - **Meaning**: RMS roll angle in degrees.

13. **YawRMS**  
    - **Format**: `XDR,A,<value>,D,YawRMS`  
    - **Meaning**: RMS yaw angle in degrees.

14. **PitchNow**  
    - **Format**: `XDR,A,<value>,D,PitchNow`  
    - **Meaning**: The **current** pitch angle in degrees at the last sample.

15. **RollNow**  
    - **Format**: `XDR,A,<value>,D,RollNow`  
    - **Meaning**: Current roll angle in degrees.

16. **YawNow**  
    - **Format**: `XDR,A,<value>,D,YawNow`  
    - **Meaning**: Current yaw (heading) angle in degrees.

### 2.6 Collision / Peak Impact

17. **PeakAccel**  
    - **Format**: `XDR,A,<value>,M/S2,PeakAccel`  
    - **Meaning**: The **peak absolute acceleration** observed in the last ~1 s window, in **m/s²**.

18. **PeakJerk**  
    - **Format**: `XDR,A,<value>,M/S3,PeakJerk`  
    - **Meaning**: The **peak jerk** (change of acceleration) in the last ~1 s, in **m/s³**.  
    - **Usage**: Another system can raise an alarm if `PeakJerk` or `PeakAccel` exceed thresholds indicating potential collision or hull damage.

---

## 3. Example NMEA Output

A typical 1 s snapshot might look like:

```
$XDR,A,0.10,HZ,Frequency*7C
$XDR,A,10.00,S,Period*21
$XDR,A,2.00,M,AvgWaveH*4F
$XDR,A,3.00,M,SigWaveH*5A
$XDR,A,0.60,M/S3,InstJerk*2A
$XDR,A,1.20,M/S2,IntJerk*01
$XDR,A,3.00,,Beaufort*7B
$XDR,A,5.00,D,PitchAmp*19
$XDR,A,6.20,D,RollAmp*BB
$XDR,A,1.00,D,YawAmp*45
$XDR,A,2.00,D,PitchRMS*55
$XDR,A,2.50,D,RollRMS*70
$XDR,A,1.00,D,YawRMS*12
$XDR,A,1.50,D,PitchNow*34
$XDR,A,1.20,D,RollNow*21
$XDR,A,-0.80,D,YawNow*5C
$XDR,A,3.20,M/S2,PeakAccel*37
$XDR,A,15.40,M/S3,PeakJerk*78
```

Each line ends with `\r\n`. The hex after `*` is the **XOR checksum**.

---

## 4. Summary for Users

- Every second, you receive **18** `$XDR` sentences over UDP broadcast, providing wave frequency/period, wave heights, jerk, collision metrics, and boat attitude angles.  
- You can parse these in your marine software (e.g., a custom logger or a standard NMEA tool) to track sea state, wave intensities, boat motion, and potential collision risk.  
- If you want to test the system **without** real waves, you can enable `simulationMode`; the device will produce a purely synthetic wave of chosen period and height, then output these same `$XDR` lines for demonstration or calibration.

Hence, the system’s NMEA messages comprehensively describe ocean wave conditions and boat dynamic behavior in an easily parsed, open‐standard format.