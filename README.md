# Cryostat_control_controller
Контроллер управления криостатом
================================
Краткое описание системы
------------------------
Контролер  управления криостатом(на схеме это контроллер обработки сигналов(далее МК2)) включен в следующую структуру.

![Alt text](image.png)


Работает это следующим образом. От Системы регулирования водоснабжения лабаратории(далее МК1) по интерфейсу SPI поступают сигналы на МК2. Сигналы поступивышие на МК2 обрабатываются формирую последовательность байтов и по интерфейсу RS232 опрашивается Криотермостат(на схеме криостат(далее КТС)). КТС выдает выдает ответ о состоянии жидости в нем(ее наличие, уровень, температуру, температуру которой она должна быть). МК2 выдает простой ответ на МК2 о состоянии жидкости означающий можно ли использовать ее оттуда или нет.


Система работает на связке микроконтроллеров серии STM32F103C8T6 (МК1) и STM32F030F4 (МК2). МК1 выполняет роль MASTER, МК2 соотвественно SLAVE. МК1 и МК2 на фото ниже

![Alt text](IMG_20220607_150920.jpg)

![Alt text](IMG_20220607_151507.jpg)

Обмен данными происходит МК2 и КТС происходит по схемам:
-------------------------------------------------------

Схема приема-передачи данных для запросов состояния у КТС:

![Alt text](<Снимок экрана 2023-10-06 090702.png>)


Схема приема-передачи данных для команд у КТС:

![Alt text](<Снимок экрана 2023-10-06 090730.png>)


Алгоритм работы МК2 на блок схеме далее:
----------------------------------------

![Alt text](<4 ПД-1.png>)


Схема подключения выводов к микроконтроллеру STM32F103C8T6
==========================================================







