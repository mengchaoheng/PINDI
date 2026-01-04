Final Experiment:
GQC:4.2.9
Experiment 1 Code ID: ad19ed32ef12db8b61af25035ae5fcb6b3393dae

_alloc_method=0:inv,
_alloc_method=1:wls,
_alloc_method=2:dir,
_alloc_method=3:pca,

1. Experiment setup:
param set-default MC_ROLL_P 6
param set-default MC_ROLLRATE_P 0.4
param set-default MC_ROLLRATE_I 0.0
param set-default MC_ROLLRATE_D 0.0
param set-default MC_ROLLRATE_FF 0

param set-default MC_PITCH_P 6
param set-default MC_PITCHRATE_P 0.4
param set-default MC_PITCHRATE_I 0.0
param set-default MC_PITCHRATE_D 0.0
param set-default MC_PITCHRATE_FF 0

param set-default MC_YAW_P 5
param set-default MC_YAWRATE_P 0.2
param set-default MC_YAWRATE_I 0.0
param set-default MC_YAWRATE_D 0
param set-default MC_YAWRATE_FF 0
param set-default MC_YAW_WEIGHT 1

param set-default USER_INDI_R_P 12
param set-default USER_INDI_P_P 12
param set-default USER_INDI_Y_P 12

1). IMU_GYRO_RATEMAX=250 default (with the same noise level as 1000 Hz, more vibration)
Noise settings:
    <plugin name='gazebo_imu_plugin' filename='libgazebo_imu_plugin.so'>
      <robotNamespace></robotNamespace>
      <linkName>ductedfan4/imu_link</linkName>
      <imuTopic>/imu</imuTopic>
      <gyroscopeNoiseDensity>0.004</gyroscopeNoiseDensity>
      <gyroscopeRandomWalk>3.8785e-04</gyroscopeRandomWalk>
      <gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
      <gyroscopeTurnOnBiasSigma>0.0087</gyroscopeTurnOnBiasSigma>
      <accelerometerNoiseDensity>0.00186</accelerometerNoiseDensity>
      <accelerometerRandomWalk>0.006</accelerometerRandomWalk>
      <accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
      <accelerometerTurnOnBiasSigma>0.196</accelerometerTurnOnBiasSigma>
    </plugin>

2). Allocation

3). INDI, lp_filter

After setting the actuator, it acts like a damping filter in simulation. Use `real_u` (if `u` is used, i.e., the command signal, it must be filtered with higher-order derivatives than `y`). Also pay attention to the range of time constants.

Experiment 1 demonstrates disturbance rejection capability and the influence of parameters on performance, focusing on actuator estimation.

Experiment 2 demonstrates PCA enhancement.

Experiment 3: real flight to reproduce Experiment 2.

===========================================================

Experiment 1:  01_43_51.ulg  
Independent of the allocator. General effect.

      <start_time>30.0</start_time>
      <running_time>150.0</running_time>

      <amplitude_x>0.1</amplitude_x>
      <frequency_x>0.2</frequency_x>
      <bias_x>0.</bias_x>

      <amplitude_y>0.1</amplitude_y>
      <frequency_y>0.4</frequency_y>
      <bias_y>0.</bias_y>

SDLOG_PROFILE=17

Filter applied to y derivative. Initial cs filter = 0, PID uses CA, all use PCA, performed in hover mode.  
External disturbance applied:  
R, P add 0.1sin(2π0.2t) and 0.1sin(2π0.4t) sinusoidal disturbance (to make the trajectory clearer).  
Initial settings: cs_cutoff=0, DGYRO_cutoff=30. INDI=1, USE_TAU=0, USER_U=1, param1.params  

After configuration and restart, parameter variations:  
USE_TAU=1, DGYRO_cutoff=20, DGYRO_cutoff=10, DGYRO_cutoff=8, cs_cutoff=20, cs_cutoff=8, cs_cutoff=1, USE_U=0 (finally restore USE_U=1, cs_cutoff=10, DGYRO_cutoff=10)  

1). pid(angle: P(12,12,10). (switch to INDI using USE_TAU)

2). Switch to degraded PINDI (actuator tau=0.03, cs_cutoff=0, DGYRO_cutoff=30), then gradually lower DGYRO_cutoff to 10 (before oscillation occurs).

3). Adjust cs_cutoff (cs_cutoff=20,10,8,3,1, DGYRO_cutoff=8) for improved PINDI (also give INDI under same conditions).  
The main source of vibration is `u` (noise and control frequency have been studied and are not discussed here), the cause is misalignment.

4). USE_U=0, under extreme delay = 0, approximately equal to PID.

Final confirmed values: cs_cutoff=10, DGYRO_cutoff=10.

Plot file: test5_2_1_1

The relative relationship between u and y affects performance. Ultimately, both should be filtered together, as it influences disturbance rejection. Extreme lag equals 0, degrading to velocity error feedback control.

Experiment 1.2:  

Plot files: test5_2_1_3 and test5_2_1_2  
Under perfectly tuned PINDI (cs_cutoff=10, DGYRO_cutoff=10), same disturbance as Experiment 1, use PCA, vary actuator time constant and B’s gain k, to observe effects on tracking performance (disturbance rejection).  
Experiment duration: 100 s. Provide statistical results. After takeoff, select parameters, then wait to enter test. After test, restore defaults and land.

      <start_time>150.0</start_time>
      <running_time>100.0</running_time>

      <amplitude_x>0.1</amplitude_x>
      <frequency_x>0.2</frequency_x>
      <bias_x>0.</bias_x>

      <amplitude_y>0.1</amplitude_y>
      <frequency_y>0.4</frequency_y>
      <bias_y>0.</bias_y>

k = 6,5,4,3,2,1 → both ends unstable (0.5 diverges, 7 near divergence). There exists an optimal value (under current controller gain and u,y estimation).  
(Start at 150 s, run for 100 s)

1. 03_44_12.ulg  
2. 03_45_56.ulg  
3. 03_47_24.ulg  
4. 03_48_28.ulg  
5. 03_50_31.ulg  
6. 03_53_22.ulg  

time = 0.1 (diverges). 0.08 0.07 0.05 0.03 0.01 0.008 ... performance worsens as time increases (start 150 s, run 100 s)  
1. 0.008  03_21_36.ulg  
2. 0.01   03_25_30.ulg  
3. 0.03   03_28_13.ulg  
4. 0.05   03_30_43.ulg  
5. 0.07   03_32_13.ulg  
6. 0.08   03_40_16.ulg  
When time constant = 0.09 → 03_33_25.ulg (near divergence)  
Angular rate and attitude tracking errors.

===========================================================

Experiment 2:  
ID: ad19ed32ef12db8b61af25035ae5fcb6b3393dae  

fisrt log: 14_53_58.ulg (all, once per type). After startup, apply disturbance, then add reference, cancel reference, change allocator, add reference again, cancel reference, repeat.  
final log: 12_02_42.ulg
Start time corresponds to event = [flag(3), flag(6), flag(9), flag(12)];

ID:

cs_cutoff=10, DGYRO_cutoff=10. dist_mag=8deg, external disturbance disabled.  
Simulated flight, model adds constant disturbance (servo adds 8deg). Proper filters applied.  
Square-wave yaw reference ±π/4, time parameter = 2 (4 s period). Directly compare INDI with and without prioritization. Initial mode: INDI + PCA.

Run test starting in hover (Loiter), then automatically enters “Position mode”.

Plot file test5_2_2: (12_02_42.ulg (test2-3.mp4) 
More test:
(14_53_58.ulg (modified PID parameters and disturbance amplitude) , 12_08_50.ulg (test2-4.mp4))

1). INDI+INV: balanced performance  
2). INDI+WLS: worse  
3). INDI+DIR: worst  
4). PINDI: enhanced  

Statistics:  
HEADLESS=1  PX4_SIM_SPEED_FACTOR=6  make px4_sitl gazebo_ductedfan4  
USER_TEST_TIME controls repetition count (100 repetitions each).  
Hover mode to test entry. During test, switch to “Position mode”. After test, remain in position mode.  
From start to reference addition and last reference cancellation, calculate tracking error based on markers. log=default.  
Set allocator first, restart, apply disturbance, add square-wave reference, wait for test end.  
Test duration:  
t_start = parameter_update(3);  
t_end = parameter_update(4);  
(May not need segmentation; directly analyze total error between t_start–t_end, though physical interpretation changes.)

Plot file: ~/Documents/UltimateTestForPINDI4/plot/test5_2_2RMS.m  
INDI+INV: 15_06_25.ulg  
INDI+WLS: 15_18_17.ulg  
INDI+DIR: 15_24_13.ulg  
PINDI: 15_29_27.ulg  

===========================================================

Experiment 3: repeat Experiment 2  
Remember to clear PX4_INFO.

Change disturbance to 8deg; actual equivalent disturbance ≈6deg, so add about 2deg.  
Actual test shows total disturbance ≈4.5deg.  
INDI k=12, corresponds to PID P=0.4 0.4 0.2. att P = 6 6 5.

Real flight: log_40_2025-6-24-14-58-42.ulg  
Others: log_36_2025-6-24-14-39-10.ulg  

Code ID: ad19ed32ef12db8b61af25035ae5fcb6b3393dae  

Log file: angular rate loop sampling frequency 285 Hz, lower than control frequency 800 Hz, limited by storage device.