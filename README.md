# Cryostat_control_controller
Контроллер управления криостатом
================================
Краткое описание системы
------------------------
Контролер  управления криостатом(на схеме это контроллер обработки сигналов(далее МК2)) включен в следующую структуру.

![Alt text](images/image.png)

Работает это следующим образом. От Системы регулирования водоснабжения лаборатории (далее МК1) по интерфейсу SPI поступают сигналы на МК2. Сигналы поступившие на МК2 обрабатываются формирую последовательность байтов и по интерфейсу RS232 опрашивается Криотермостат (на схеме криостат (далее КТС)). КТС выдает ответ о состоянии жидкости в нем (ее наличие, уровень, температуру, температуру которой она должна быть). МК2 выдает простой ответ на МК2 о состоянии жидкости означающий можно ли использовать ее оттуда или нет.

Система работает на связке микроконтроллеров серии STM32F103C8T6 (МК1) и STM32F030F4 (МК2). 
МК1 выполняет роль MASTER,

![Alt text](images/IMG_20220607_150920.jpg)

МК2 соответственно SLAVE.

![Alt text](images/IMG_20220607_151507.jpg)

Обмен данными между МК2 и КТС происходит по схемам:
-------------------------------------------------------

Схема приема-передачи данных для запросов состояния у КТС:

![Alt text](images/zap.png)

Схема приема-передачи данных для команд у КТС:

![Alt text](images/command.png)

Алгоритм работы МК2 на блок схеме далее:
----------------------------------------

![Alt text](images/pd.png)

Схема подключения выводов к МК2
==========================================================
![Alt text](images/image1.png)

Схема подключения МК2 к преобразователю USART-RS232
==========================================================

![Alt text](images/image2.png)
