DNN KWS on STM32F4
==================

Use STM32F407's 12 bit ADC to get audio data from a microphone, and then run DNN KWS to detect speech command.

### Hardware 
+ [Arch Max (STM32F407)](https://www.seeedstudio.com/Arch-Max-v1.1-p-2632.html)
+ [Grove Sound Sensor (must add a bias for the microphone)](https://github.com/xiongyihui/ML-KWS-for-MCU/issues/1)

```
           TIM3 (16K)
              \
Microphone -> ADC (PC0) -> DMA
```

![](https://statics3.seeedstudio.com/images/product/102080004%200.jpg)


### Build & Run

In this example, the KWS inference is run on the audio data from ADC.
First create a new project and install any python dependencies prompted when project is created for the first time after the installation of mbed-cli.
```bash
mbed new kws_simple_test --mbedlib 
```
Fetch the required mbed libraries for compilation.
```bash
cd kws_simple_test
mbed deploy
```
Compile the code for the mbed board (for example Arch Max STM32F407).
```bash
mbed compile -m ARCH_MAX -t GCC_ARM --source . --source ../Source --source ../Examples/stm32f4_realtime_test --source ../CMSIS_5/CMSIS/NN/Include --source ../CMSIS_5/CMSIS/NN/Source --source ../CMSIS_5/CMSIS/DSP/Include --source ../CMSIS_5/CMSIS/DSP/Source   --source ../CMSIS_5/CMSIS/Core/Include   --profile ../release_O3.json
```

Copy the binary (.bin) to the board (Make sure the board is detected and mounted). Open a serial terminal (e.g. putty or minicom) and see the final classification output on screen. 
```bash
cp ./BUILD/ARCH_MAX/GCC_ARM/kws_simple_test.bin /media/$USER/MBED
sudo minicom -D /dev/ttyACM0 -b 576000
```
