
# 0. M5Stack AtomS3 WiFi connection first!

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

### User-perspective Summary

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

## A. General NMEA XDR Format

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

## B. Full List of Output Sentences

Below is a **complete** set of XDR messages sent by the code. Each line’s `<label>` matches the code’s `sendNmeaXDR(...)` call, and `<unit>` corresponds to the parameter’s dimension.

### B.1 Wave Frequency & Period

1. **Frequency**  
   - **Format**: `XDR,A,<value>,HZ,Frequency`  
   - **Meaning**: The dominant wave frequency in **Hertz** (cycles per second), derived by zero‐cross counting on filtered acceleration.  
   - **Typical Range**: ~0.03–1.0 Hz for ocean waves, depending on wave periods from ~1 s to ~30 s.

2. **Period**  
   - **Format**: `XDR,A,<value>,S,Period`  
   - **Meaning**: The wave’s fundamental period in **seconds** (1 / frequency).  
   - **Typical Range**: ~1–30 s for typical seas.

### B.2 Wave Heights

3. **AvgWaveH**  
   - **Format**: `XDR,A,<value>,M,AvgWaveH`  
   - **Meaning**: The **average** crest‐to‐trough wave height in **meters**, computed over the last ring buffer.  
   - **Typical Range**: < 0.1 m in calm seas up to several meters in higher seas.

4. **SigWaveH**  
   - **Format**: `XDR,A,<value>,M,SigWaveH`  
   - **Meaning**: The **significant** wave height in **meters** (average height of top 1/3 largest waves). Often considered a key oceanographic measure of sea state.

### B.3 Slamming / Jerk

5. **InstJerk**  
   - **Format**: `XDR,A,<value>,M/S3,InstJerk`  
   - **Meaning**: The **instantaneous jerk** (rate of acceleration change) at the latest sample, in **m/s³**.  
   - **Typical Range**: 0–many tens m/s³ if the boat slams abruptly.

6. **IntJerk**  
   - **Format**: `XDR,A,<value>,M/S2,IntJerk`  
   - **Meaning**: The integrated absolute jerk over the entire buffer, in **m/s²** (dimension is effectively jerk × time). Helpful as a “slamming severity” measure.

### B.4 Beaufort

7. **Beaufort**  
   - **Format**: `XDR,A,<value>,,Beaufort`  
   - **Meaning**: An integer (0–12 or more) representing an approximate **Beaufort scale** level, derived from wave height.  
   - **Typical Range**: 0 = calm, 12 = hurricane‐force conditions.  
   - **Unit**: none, so `<unit>` is empty.

### B.5 Attitude Motion

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

### B.6 Collision / Peak Impact

17. **PeakAccel**  
    - **Format**: `XDR,A,<value>,M/S2,PeakAccel`  
    - **Meaning**: The **peak absolute acceleration** observed in the last ~1 s window, in **m/s²**.

18. **PeakJerk**  
    - **Format**: `XDR,A,<value>,M/S3,PeakJerk`  
    - **Meaning**: The **peak jerk** (change of acceleration) in the last ~1 s, in **m/s³**.  
    - **Usage**: Another system can raise an alarm if `PeakJerk` or `PeakAccel` exceed thresholds indicating potential collision or hull damage.

---

## C. Example NMEA Output

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

## D. Summary for Users

- Every second, you receive **18** `$XDR` sentences over UDP broadcast, providing wave frequency/period, wave heights, jerk, collision metrics, and boat attitude angles.  
- You can parse these in your marine software (e.g., a custom logger or a standard NMEA tool) to track sea state, wave intensities, boat motion, and potential collision risk.  
- If you want to test the system **without** real waves, you can enable `simulationMode`; the device will produce a purely synthetic wave of chosen period and height, then output these same `$XDR` lines for demonstration or calibration.

Hence, the system’s NMEA messages comprehensively describe ocean wave conditions and boat dynamic behavior in an easily parsed, open‐standard format.


---

# Wave, Slamming, and Attitude Monitoring Systems: the Theory

This project runs on the M5Atom S3 and is designed to monitor a boat’s vertical motion (heave), wave frequency and height, rapid motion changes (jerk) associated with slamming or collisions, and the boat’s orientation (pitch, roll, and yaw). Data are displayed on the device’s built-in LCD and broadcast over Wi‑Fi in standard NMEA 0183 XDR sentences.

An optional simulation mode is provided so that users can test the system with a synthetic sinusoidal wave defined by a specified period and crest‑to‑trough height.

This section describes the mathematical principles behind the data processing, filtering, and parameter estimation used in the code.

---

## AB. Signal Acquisition and Conversion

### 1.1 IMU Data

- **Accelerometer Data:**  
  The sensor returns acceleration values in units of “g” (gravitational acceleration). These are later converted to m/s² by multiplying by the standard gravity (9.80665 m/s²).

- **Gyroscope Data:**  
  The gyroscope produces angular velocity in degrees per second. Before being used by the Madgwick filter, these values are converted to radians per second:
  \[
  \omega_{\text{rad/s}} = \omega_{\text{deg/s}} \times \frac{\pi}{180}
  \]

- **Madgwick Filter:**  
  The filter fuses the accelerometer and gyroscope data to compute an orientation quaternion. From this quaternion, the filter extracts pitch, roll, and yaw angles (returned in degrees).  
  *Note:* While the input (gyro) is converted to rad/s, the output angles are provided in degrees for human readability.

---

## 2. Band-Pass Filtering

### 2.1 Purpose

The raw vertical acceleration data contain unwanted components:
- **Low-frequency drift:** Caused by sensor bias or slow changes in orientation.
- **High-frequency noise:** Resulting from mechanical vibrations or sensor noise.

A **band‑pass filter** is applied to retain only the frequencies associated with typical ocean waves (for example, between 0.03 Hz and 1.0 Hz).

### 2.2 Filter Design

Two 2nd‑order biquad filters are cascaded:
- **High-Pass Filter:**  
  Removes frequencies below the low cutoff (e.g., 0.03 Hz). Its continuous‑time transfer function is of the form:
  \[
  H_{\text{HP}}(s) = \frac{s^2}{s^2 + \frac{\omega_0}{Q}\, s + \omega_0^2}
  \]
  where \(\omega_0 = 2\pi \cdot \text{LOW\_CUTOFF\_HZ}\) and \(Q\) is the quality factor (typically 0.707 for a Butterworth response).

- **Low-Pass Filter:**  
  Removes frequencies above the high cutoff (e.g., 1.0 Hz) with a transfer function:
  \[
  H_{\text{LP}}(s) = \frac{\omega_0^2}{s^2 + \frac{\omega_0}{Q}\, s + \omega_0^2}
  \]
  where here \(\omega_0 = 2\pi \cdot \text{HIGH\_CUTOFF\_HZ}\).

These filters are implemented in discrete time via difference equations (Direct Form II) in our code.

---

## 3. Integration and Wave Parameter Estimation

### 3.1 Integration

After filtering, the acceleration data are integrated twice to estimate:
- **Velocity:**  
  \[
  v[k] = v[k-1] + a[k] \cdot \Delta t
  \]
- **Displacement (Heave):**  
  \[
  d[k] = d[k-1] + v[k] \cdot \Delta t
  \]
  
Small biases in acceleration can integrate to significant displacement errors, so clamping is used if velocity or displacement exceed set limits.

### 3.2 Wave Frequency Estimation

The dominant wave frequency is determined using **zero‑cross detection** on the filtered acceleration data:
- The average value is subtracted from the signal.
- The code detects negative‑to‑positive zero crossings.
- The average number of samples between crossings is computed and, with the known sampling frequency, converted to a period \(T\) and frequency \(f = \frac{1}{T}\).

### 3.3 Wave Height Calculation

The integrated displacement signal is analyzed to determine wave heights:
- The signal is centered by subtracting its mean.
- Zero‑crossing detection on the displacement signal segments the data into individual wave cycles.
- For each cycle, the local minimum and maximum are found.
- **Average Wave Height (AH):** The mean crest‑to‑trough amplitude across all cycles.
- **Significant Wave Height (SH):** The average amplitude of the largest one‑third of cycles.

In the case of an ideal sinusoid \(y(t) = A \sin(\omega t)\), the crest‑to‑trough distance is \(2A\).

---

## 4. Slamming (Jerk) and Collision Metrics

### 4.1 Jerk Calculation

- **Instantaneous Jerk:**  
  Jerk is defined as the derivative of acceleration:
  \[
  j[k] \approx \frac{a[k] - a[k-1]}{\Delta t}
  \]
  This value is computed using the most recent two samples.

- **Integrated Jerk:**  
  The system also computes the sum of the absolute jerk values over the ring buffer (scaled by the time interval), providing an overall measure of “slamming” intensity.

### 4.2 Collision Detection

In addition, the code scans the most recent 1‑second segment of data to find:
- **Peak Acceleration (PeakAccel)**
- **Peak Jerk (PeakJerk)**
  
These parameters serve as indicators of potentially damaging collisions or slamming events on the hull.

---

## 5. Attitude (Orientation) Estimation

### 5.1 Madgwick Filter

The Madgwick filter fuses accelerometer and gyro data to calculate the orientation (in quaternions). From these quaternions, the filter computes:
- **Pitch:** Rotation about the sensor’s X‑axis.
- **Roll:** Rotation about the sensor’s Y‑axis.
- **Yaw:** Rotation about the sensor’s Z‑axis.

### 5.2 Attitude Metrics

- **Peak-to-Peak Amplitude:**  
  The maximum difference in pitch, roll, and yaw over the ring buffer.
- **RMS Values:**  
  The root‑mean‑square of the pitch, roll, and yaw values over time.
- **Current Angles:**  
  The most recent pitch, roll, and yaw values.

*Note:* Because gyroscopes provide angular velocity, the Madgwick filter integrates these values (while fusing them with accelerometer data) to compute absolute angles. The code converts the gyro values from deg/s to rad/s before passing them to the filter; the filter then outputs the orientation angles in degrees.

---


## 7. Mathematical Principles Summary

- **Filtering:**  
  The band‑pass filter uses two cascaded 2nd‑order biquad filters to remove both very low and very high frequencies. This preserves the frequency components associated with ocean waves.
  
- **Integration:**  
  Acceleration is integrated over time to yield velocity and then displacement. In a simple sinusoid \(x(t) = A \sin(\omega t)\), the acceleration is \(a(t) = -\omega^2 A \sin(\omega t)\). Thus, the amplitude \(A\) (and hence the crest‑to‑trough wave height of \(2A\)) can be estimated from the acceleration if \(\omega\) is known.

- **Zero-Cross Detection:**  
  Both the wave frequency and the wave height calculations rely on detecting zero crossings (when the signal goes from negative to positive). The average interval between these crossings yields the wave period; the difference between local minima and maxima (between crossings) gives the wave height.

- **Jerk:**  
  Jerk is calculated as the discrete derivative of acceleration. Instantaneous jerk uses two successive samples, while integrated jerk sums the absolute values over the measurement window.

- **Attitude Estimation:**  
  The Madgwick filter uses a quaternion-based algorithm to fuse accelerometer and gyro data. Gyro values (converted to rad/s) are integrated, and an error-correction term (scaled by a gain parameter) is applied to compute pitch, roll, and yaw.

- **NMEA Output:**  
  The computed values are then formatted in NMEA 0183 XDR sentences, a standard marine data format, for integration with other shipboard systems.

---

## 8. Usage Summary for End Users

- **Mount the Device:** Place the M5Atom S3 so that its axes align as closely as possible with the boat’s forward, starboard, and down directions.
- **WiFi Configuration:** At startup, you can press the button within 5 seconds to upload new WiFi credentials. These credentials are stored non‑volatilely.
- **Operation:** The system displays key parameters on the onboard LCD and simultaneously broadcasts NMEA messages. Your marine software can read these messages to monitor sea state, detect slamming events, and assess the boat’s attitude.
- **Calibration:** Although the system applies advanced filtering and integration, users may need to ensure proper sensor orientation. If the pitch, roll, and yaw outputs do not match expectations, verify that the sensor is mounted correctly or apply software corrections.

## Dependencies

- [M5Atom Library](https://github.com/m5stack/M5Atom)  
- WiFi and WebServer libraries (included in the Arduino core for ESP32)  
- Preferences library for storing data in non‑volatile memory

## License

This project is provided under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Below is a **high‐level user guide** to the **Heave + Wave + Collision** measurement system, describing **what** the program does, **how** it presents results, and **which NMEA outputs** it sends. It's written for **end users** (e.g., boat operators or marine researchers) rather than developers diving into code details.
