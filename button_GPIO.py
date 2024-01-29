import RPi.GPIO as GPIO
import subprocess

# Установка режима нумерации GPIO в соответствии с BCM
GPIO.setmode(GPIO.BCM)

# Номера пинов, к которым подключены сигналы и лампочка
input_pin = 14
output_pin = 15

# Настройка пинов
GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(output_pin, GPIO.OUT)


# Функция, которая будет выполняться при срабатывании прерывания от входного пина
def handle_interrupt(channel):
    print("Сигнал принят!")

    # Запуск скрипта run.sh с помощью subprocess
    subprocess.call("./run.sh", shell=True)

    # Отправка сигнала на выходной пин для сигнализации
    GPIO.output(output_pin, GPIO.HIGH)


# Настройка прерывания на входном пине
GPIO.add_event_detect(input_pin, GPIO.FALLING, callback=handle_interrupt, bouncetime=200)

# Основной цикл программы
try:
    while True:
        pass

except KeyboardInterrupt:
    print("Программа остановлена")

finally:
    # Сброс пинов и очистка ресурсов GPIO
    GPIO.cleanup()