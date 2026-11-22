# 🏎️ Robot Autónomo — WRO Futuros Ingenieros 2026

Código fuente en **C++ (Arduino)** para un vehículo autónomo de la categoría **WRO Future Engineers** (Reto Abierto), diseñado bajo una arquitectura distribuida **Maestro - Esclavo** comunicada por bus **I2C**.

- **`maestro.cpp`:** Nodo principal de procesamiento y navegación. Gestiona el protocolo de arranque por pulsador, el filtrado exponencial de 3 sensores ultrasónicos (`NewPing`), la medición angular mediante giroscopio **MPU6050** (`Wire`), la **Máquina de Estados Finitos** (`RECTA`, `CURVA`, `COOLDOWN`, `APROXIMACION`, `FIN`), la detección automática del sentido de pista y el controlador **PD** de centrado en carril.
- **`esclavo.cpp`:** Nodo dedicado de actuación (dirección I2C `0x08`). Recibe las consignas de movimiento desde el maestro y controla en tiempo real el **servomotor de dirección** (`Servo`) y la velocidad por PWM del **motor DC de tracción**.