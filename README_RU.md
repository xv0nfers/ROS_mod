# Стек зрения и наведения USV (ROS 2 Humble)

Этот проект оформлен как бортовой стек для автономного USV с акцентом на **надежность, детерминизм и корректную матмодель управления**, а не на избыточное усложнение машинного зрения.

## Инженерная позиция

Ключевая сложность — **правильный синтез управляющих воздействий**, а не усложнение перцепции.

- Зрение — это источник входных данных, а не «мозг» системы.
- Предпочтение простым, устойчивым и универсальным алгоритмам.
- OpenCV-подходы подходят для embedded-устройств даже без NPU.
- Нейросети/NPU — опциональный инструмент, не обязательное условие архитектуры.
- Основной фокус: фильтрация, fusion с телеметрией, подавление автоколебаний.

---

## 1) Установка на бортовой компьютер USV

### Требования

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- OpenCV (через систему или pip):
  - `python3-opencv` (желательно для embedded Linux)
  - или `opencv-python`

### Подготовка workspace

```bash
mkdir -p ~/usv_ws/src
cd ~/usv_ws/src
git clone <YOUR_REPO_URL> ROS_mod
cd ROS_mod

# Python зависимости
pip3 install -r requirements.txt

# Системный OpenCV (рекомендуется)
sudo apt-get update
sudo apt-get install -y python3-opencv

# ROS зависимости + сборка
cd ~/usv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/usv_ws/install/setup.bash
```

Опционально добавить в `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/usv_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2) Запуск на USV

### Запуск перцепции

```bash
ros2 launch yolo_bringup yolo.launch.py
```

### (Опционально) OpenCV-only контур

Для максимально предсказуемой и легкой обработки запустите OpenCV-узел:

```bash
ros2 run yolo_ros usv_opencv_tracker_node
```

Узел отслеживает движение признаков в нижней (водной) части кадра и публикует легковесный вектор наведения в `/usv/target_vector`.

### Базовые топики интеграции

- Камера: `/camera/image_raw`
- Трекинг/детекции: `/tracking` (или топик вашего OpenCV-трекера)
- Вектор наведения (если используется selector-узел): `/usv/target_vector`
- Отладочное изображение: `/usv/vision_debug`

---

## 3) Обучение (если используются нейромодели)

> Обучение нейросети не должно заменять качество математической модели управления.

### Рекомендации по датасету

- Классы USV/boat/canoe
- Сложные морские условия: блики, пена, волна, дымка, дождь
- Баланс дальних и ближних дистанций

### Пример обучения

```bash
yolo detect train \
  model=yolov8n.pt \
  data=usv_dataset.yaml \
  imgsz=640 \
  epochs=100 \
  batch=32 \
  device=0
```

### Экспорт под edge

```bash
yolo export model=best.pt format=onnx imgsz=640
```

Далее — конвертация/калибровка под целевой runtime (INT8, вендор-специфично для NPU).

---

## 4) Критично для контура управления

Чтобы избежать срыва и автоколебаний:
- Ограничивайте и нормируйте управляющие команды.
- Делайте фильтрацию с явным учетом задержек.
- Объединяйте визуальные наблюдения с IMU/GNSS/курс/скорость.
- Используйте anti-windup и gain scheduling.
- Держите деградированный fallback-режим при падении качества перцепции.

---

## 5) Ссылки OpenCV и ROS

- OpenCV: https://opencv.org/
- ROS wiki `vision_opencv`: https://wiki.ros.org/vision_opencv
- Репозиторий `vision_opencv`: https://github.com/ros-perception/vision_opencv
- Пример ROS2 + OpenCV: https://github.com/luciapferreira/ros2-opencv-object-detection

---

## 6) Быстрый старт

```bash
# Терминал 1
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 launch yolo_bringup yolo.launch.py

# Терминал 2
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 run yolo_ros usv_opencv_tracker_node
```
