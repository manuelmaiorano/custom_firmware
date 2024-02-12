# Implementazione filtro di Kalman e driver di comunicazione per droni Crazyflie

Il repository è strutturato nel seguente modo:

- Nella cartella CMSIS-DSP sono presenti le implementazioni delle principali funzioni matematiche utilizzate dal filtro di Kalman
- Nella cartella Drivers/CMSIS è presente l'implementazione di funzioni e macro specifiche per l'architettura ARM Cortex-M7
- Nella cartella Drivers/MPU6050 è presente il driver per l'IMU MPU6050: [link](https://github.com/libdriver/mpu6050)
- Nella cartella Drivers/STM32F7xx_HAL_Driver sono presenti i driver HAL per la comunicazione con le periferiche
- Nella cartella Third_Party/FreeRTOS è presente l'implementazione del sistema operativo real-time FreeRTOS
- Nella cartella Third_Party/SEGGER è presente la libreria di logging per comunicare con il software Segger SystemView
- Nella cartella vendor/libdw1000 è presente il driver per comunicare con l'antenna dw1000 presente sul Loco Positioning Deck
- Il file Makefile e la cartella cm-makefile definiscono la procedura di build del codice

Nella cartella src è presente l'implementazione dei driver per le periferiche I2C e SPI e l'implementazione del filtro di Kalman:

- I file spi_driver.c e i2c_driver.c implementano l'API dei driver definita nei corrispondenti file .h presenti nella cartella include
- I file lpsTdoa2Tag.c, tdoaEngine.c, tdoaEngineInstance.c, tdoaStorage.c e clockCorrectionEngine.c implementano il protocollo TDoA2.
- Il file locodeck.c definisce l'implemetazione del task responsabile della comunicazione con il Loco Positioning Deck.
- Il file sensor_task.c definisce l'implemetazione del task responsabile della comunicazione con il sensore MPU6050.
- Il file estimator.c definisce l'implemetazione del task che aggiorna periodicamente la stima dello stato tramite filtro di Kalman. Utilizza le funzioni implementate in kalman_core.c (fase di predizione e update generali), mm_tdoa.c (modello dei sensori specifico del protocollo TDoA)
