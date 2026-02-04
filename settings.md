# Experiment

Experiment 1 is used to illustrate the disturbance rejection capability and the impact of parameters on PINDI performance, with a focus on actuator estimation (filtering). Experiment 2 is used to demonstrate the performance enhancement of PCA. Experiment 3 is an actual flight test used to reproduce Experiment 2.


1.试验设置：
```
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
```

1. Noise
   
Set the parameter IMU_GYRO_RATEMAX=250 and set the IMU parameters in [SDF files of ductedfan4](https://github.com/mengchaoheng/DF_gazebo/blob/ae5619cd3821529ff4a8e816a8f7c5bed311d7c0/models/ductedfan4/ductedfan4.sdf.jinja#L1209-L1220):

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

2. Allocation

Control allocation algorithm and corresponding parameters：

```
_alloc_method=0:inv,
_alloc_method=1:wls,
_alloc_method=2:dir,
_alloc_method=3:pca,
```

3. Filter

After setting up the actuator model, it has a similar filtering effect in the simulation. In the simulation, we use the real actuator command (real_u) as the feedback for INDI. If the actuator command is used instead of the actual deflection value, the filter must have a much higher cutoff frequency than the higher-order derivatives of y. Also, pay attention to the time constant range.



## Experiment 1 

In fact, this experiment is unrelated to the allocator. It belongs to the general conclusions of INDI.

During the experiment, disturbances are introduced through a [plugin](https://github.com/mengchaoheng/DF_gazebo/blob/ae5619cd3821529ff4a8e816a8f7c5bed311d7c0/src/torque_disturbance_plugin.cpp) with the following [parameter settings](https://github.com/mengchaoheng/DF_gazebo/blob/ae5619cd3821529ff4a8e816a8f7c5bed311d7c0/models/ductedfan4/ductedfan4.sdf.jinja#L1168-L1183):

```xml
<start_time>30.0</start_time>
<running_time>150.0</running_time>

<amplitude_x>0.1</amplitude_x>
<frequency_x>0.2</frequency_x>
<bias_x>0.</bias_x>

<amplitude_y>0.1</amplitude_y>
<frequency_y>0.4</frequency_y>
<bias_y>0.</bias_y>

```
Apply external disturbances with 0.1sin(2*pi*0.2t) and 0.1sin(2*pi*0.4t) in the Roll and Pitch channels respectively. The experiment is conducted in the fixed position hover mode.

### Experiment 1.1
We filter the derivative of y, which is the angular acceleration (output), and the actuator command (input) separately. The cutoff frequencies are regulated by the parameters DGYRO_cutoff and cs_cutoff.

Initially, cs_cutoff=0 means no input filtering is used, and the CA module is used for all controllers (the built-in mixer is shielded).

Initial Setup:

```
cs_cutoff=0
DGYRO_cutoff=30
INDI=1
USE_TAU=0
USER_U=1,
```

Parameter change process：

```
(1). USE_TAU=1
(2). DGYRO_cutoff=20
(3). DGYRO_cutoff=10
(4). DGYRO_cutoff=8
(5). cs_cutoff=20
(6). cs_cutoff=8
(7). cs_cutoff=1
(8). USE_U=0
```

In fact, the aforementioned steps correspond to the following status:

1. Initially, it is equivalent to using a PID controller, then we switch to INDI by setting USE_TAU=1.

2. (1--4), Using a PINDI controller without parameter tuning, with the executor time parameter tau=0.03, cs_cutoff=0, DGYRO_cutoff=30, gradually reducing DGYRO_cutoff to 10 (the value before jitter appears).

3. (5--7), Adjust cs_cutoff (cs_cutoff=20, 10, 8, 3, 1) while keeping DGYRO_cutoff=8 to improve the PINDI performance. 


4. (8), Setting USE_U=0 indicates that under extreme delay, the controller approaches a PID control.

Finally, we determined the optimal parameters:

```
cs_cutoff=10
DGYRO_cutoff=10.
```

### Experiment 1.2

The filter cutoff frequency and disturbance settings are the same as Experiment 1.1, while changing the value of the parameter k in the control effect matrix， observe its effect on tracking performance (disturbance suppression).

Note that the experiment starts at 150s and runs for 100s:
```xml
    <start_time>150.0</start_time>
    <running_time>100.0</running_time>

    <amplitude_x>0.1</amplitude_x>
    <frequency_x>0.2</frequency_x>
    <bias_x>0.</bias_x>

    <amplitude_y>0.1</amplitude_y>
    <frequency_y>0.4</frequency_y>
    <bias_y>0.</bias_y>
```

After takeoff, select the parameters, then wait to enter the test. The test duration is 100 seconds. Calculate the 2-norm of the tracking error for each data point and plot a boxplot of these error norms. After the test, restore the default parameters and land.

We found that the system approaches divergence when k=7, and also diverges when k=0.5. The system can only operate normally when k=6, 5, 4, 3, 2, 1. This indicates that there is an optimal k under the current controller gain and u, y estimates.

Flight Log:

```
1. k=1， 03_44_12.ulg
2. k=2， 03_45_56.ulg
3. k=3， 03_47_24.ulg
4. k=4， 03_48_28.ulg
5. k=5， 03_50_31.ulg
6. k=6， 03_53_22.ulg
```

### Experiment 1.3
All settings are the same as in Experiment 1.2, but changed to observe the effect of the actuator time constant on tracking performance. We found that the system diverges when T_s=0.1. When T_s=0.08, 0.07, 0.05, 0.03, 0.01, 0.008, the system operates normally, and the larger the T_s, the worse the tracking performance.

Flight Log:
```
1. T_s=0.008， 03_21_36.ulg
2. T_s=0.01，  03_25_30.ulg
3. T_s=0.03，  03_28_13.ulg
4. T_s=0.05，  03_30_43.ulg
5. T_s=0.07，  03_32_13.ulg
6. T_s=0.08，  03_40_16.ulg    
```
Additionally, it is worth mentioning that when the time constant is set to T_s=0.09, the system approaches instability, as indicated in the flight log: 03_33_25.ulg.

## Experiment 2

With the same filter parameter settings, start the aircraft to a certain altitude and make it hover stably. Then introduce actuator disturbance dist_mag=8 deg, wait for the system to stabilize again, and input the heading reference signal. For each control allocation algorithm, observe the tracking performance of the same navigation reference signal. Since the experimental process is: input heading reference, cancel the heading reference after one cycle, modify the allocation algorithm, and repeat, the time to track the reference signal is recorded in the event flags [flag(3), flag(6), flag(9), flag(12)].

Note that there are no external disturbances during the experiment; disturbances are added through the actuator u_dist. The heading reference signal is a square wave with amplitude pi/4 and period 4 seconds (time parameter = 2). It directly enters Loiter mode and automatically switches to position mode after the experiment starts.

The relevant flight logs: 14_53_58.ulg, 12_02_42.ulg, 12_08_50.ulg, 06_27_18.ulg.


When calculating RMS, use the HEADLESS mode of px4 gazebo simulation and accelerate the run with PX4_SIM_SPEED_FACTOR=6. Terminal command:
```sh
HEADLESS=1  PX4_SIM_SPEED_FACTOR=6  make px4_sitl gazebo_ductedfan4
```

Set the number of repetitions USER_TEST_TIME=100. Enter the test from hover mode, and during the test, switch to position mode, keeping it in position mode after completion. According to the flag indicating parameter changes in px4, record the tracking error from the start, then adding ref, to the last time ref is canceled. Since the allocation method is set first, then after restart, disturbance is added, followed by a square wave reference signal, waiting for the test to end. Therefore, the test time is between: t_start = parameter_update(3); t_end = parameter_update(4);

Flight Log:
```
INDI+inv ：   15_06_25.ulg
INDI+wls      15_18_17.ulg
INDI+DIR：    15_24_13.ulg
PINDI：     15_29_27.ulg
```
 
### Experiment 3
The actual flight test aims to reproduce the content of Experiment 2. Since it was found during actual flight that the rudder had a 6deg deviation without any disturbance, adding 2deg was considered. Actual testing showed that the same effect as the simulation occurred when the disturbance was 4.5deg.

Note that the controller gain of INDI is set to
```
param set-default USER_INDI_R_P 12
param set-default USER_INDI_P_P 12
param set-default USER_INDI_Y_P 12
```

Corresponding to PID:
```
MC_ROLLRATE_P=0.4
MC_PITCHRATE_P=0.4 
MC_YAWRATE_P=0.2
```

The attitude controller gains are 
```
MC_ROLL_P=6
MC_PITCH_P=6
MC_YAW_P=5.
```

Flight Log:
```
log_40_2025-6-24-14-58-42.ulg
log_36_2025-6-24-14-39-10.ulg
```



