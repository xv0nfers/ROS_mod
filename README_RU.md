# Автономный стек перехвата целей для БЭК/БНА (ROS 2 Humble)

Это бортовой модуль восприятия и выбора цели для БЭК.
Проект адаптирован под **морские задачи преследования и перехвата** на ограниченном вычислительном железе.

## 1) Назначение системы

Система обеспечивает:
- Интеграцию детекции/трекинга (YOLO + ByteTrack) в реальном времени.
- Морскую фильтрацию целей (игнор верхней части кадра с небом/облаками).
- Логику lock-on по стабильному tracking ID.
- Выдачу вектора наведения для автопилота.
- Отладочный видеопоток для контроля оператором.

---

## 2) Структура репозитория

- `yolo_ros/` — ROS 2 Python-пакет с узлами инференса и трекинга.
- `yolo_msgs/` — пользовательские типы сообщений.
- `yolo_bringup/` — launch-файлы запуска.
- `yolo_ros/yolo_ros/usv_target_selector_node.py` — узел выбора/фиксации цели и расчета наведения.

---

## 3) Установка на бортовой компьютер USV

### 3.1 Требования

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- Драйвер камеры, публикующий `sensor_msgs/Image`
- (Опционально) NPU runtime/SDK производителя для INT8

### 3.2 Подготовка workspace

```bash
mkdir -p ~/usv_ws/src
cd ~/usv_ws/src
git clone <ВАШ-URL-РЕПОЗИТОРИЯ> ROS_mod
cd ROS_mod
pip3 install -r requirements.txt
cd ~/usv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/usv_ws/install/setup.bash
```

### 3.3 Автозагрузка окружения (рекомендуется)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/usv_ws/install/setup.bash" >> ~/.bashrc
```

---

## 4) Запуск на БЭК

### 4.1 Запуск перцепции (YOLO + tracking)

```bash
ros2 launch yolo_bringup yolov8.launch.py
```

Или через общий launch с параметрами:

```bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=<ПУТЬ_ИЛИ_ИМЯ_МОДЕЛИ> \
  device:=cpu \
  use_tracking:=True \
  use_debug:=True
```

### 4.2 Запуск узла выбора цели

```bash
ros2 run yolo_ros usv_target_selector_node
```

### 4.3 Основные топики интеграции

Вход:
- `/camera/image_raw` (`sensor_msgs/Image`)
- `/tracking` (`vision_msgs/Detection2DArray`)

Выход:
- `/usv/target_vector` (`geometry_msgs/Twist`)
  - `linear.x` — нормализованная ошибка азимута `[-1..1]`
  - `linear.y` — эвристика «дистанции»
  - `linear.z` — флаг захвата (`1.0`/`0.0`)
- `/usv/vision_debug` (`sensor_msgs/Image`)

### 4.4 Параметры для морской настройки

- `roi_top_ratio` (по умолчанию `0.30`) — игнор верхней части кадра.
- `priority_class_ids` (по умолчанию `[0,1]`) — приоритетные классы целей.
- `min_stable_hits` (по умолчанию `3`) — минимум попаданий ID до захвата.
- `max_lock_misses` (по умолчанию `5`) — допустимые пропуски перед сбросом захвата.
- `max_detection_age_sec` (по умолчанию `0.25`) — отсев устаревших детекций.

Пример запуска с параметрами:

```bash
ros2 run yolo_ros usv_target_selector_node --ros-args \
  -p roi_top_ratio:=0.30 \
  -p priority_class_ids:="[0,1]" \
  -p min_stable_hits:=4 \
  -p max_lock_misses:=6
```

---

## 5) Обучение модели под морские цели

Обучение выполняется оффлайн (рабочая станция/сервер), на БЭК разворачивается уже экспортированная модель.

### 5.1 Рекомендации к датасету

Собирать и размечать:
- USV/лодки под разными ракурсами и дистанциями.
- Малые цели (каноэ, надувные лодки).
- Сложные условия: блики, волна, пена, туман, дождь, закат/контровый свет.

Рекомендуемые классы:
- `0: usv`
- `1: boat`

### 5.2 Пример обучения

```bash
yolo detect train \
  model=yolov8n.pt \
  data=usv_dataset.yaml \
  imgsz=640 \
  epochs=100 \
  batch=32 \
  device=0
```

### 5.3 Экспорт под edge/NPU

```bash
yolo export model=best.pt format=onnx imgsz=640
```

Далее — конвертация/калибровка в INT8 через инструменты вашего NPU-вендора.

### 5.4 Чеклист перед выходом в море

- Проверить соответствие `class_id` и `priority_class_ids`.
- Проверить фактический FPS (целевой 30 FPS).
- Убедиться, что трекер публикует стабильный `tracking ID`.
- Выполнить испытания и тонкую настройку параметров lock-on.

---

## 6) Эксплуатационные рекомендации

- Обязательно иметь ручной override/дистанционное отключение автономного режима.
- Писать `ros2 bag` на ходовых испытаниях для последующей донастройки.
- Пере-проверять систему после обновления модели, камеры или прошивки.

---

## 7) Быстрый старт

```bash
# Терминал 1
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 launch yolo_bringup yolov8.launch.py

# Терминал 2
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 run yolo_ros usv_target_selector_node
```
