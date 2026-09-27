# Инструкция для жюри: запуск и проверка решения LCM Solution

Решение - пакет ROS 2 Humble `tram_odometry` (Python) + приложенный пакет сообщений `tram_vehicle_msgs`.
Готовый Docker-образ опубликован: **`ghcr.io/tellsamm/lcm-odometry:latest`** (x86_64 и arm64). Собирать ничего не нужно.

## Самый быстрый путь: 3 команды

Нужен только Docker (Docker Desktop на Windows/macOS или Docker Engine на Linux) и распакованные bag-файлы.

Шаг 1 - скачать репозиторий (если он уже скачан, перейдите к шагу 2):
```bash
git clone https://github.com/TellSamm/LCM-Solution.git
cd LCM-Solution
```

Шаг 2 - запустить проверку, указав путь к папке **одного прогона на вашем диске** - той, в которой лежат
`<имя>_0.db3` и `metadata.yaml`. Ниже `ПУТЬ_К_ПРОГОНУ` нужно заменить на ваш путь:

Linux / macOS / Git Bash:
```bash
./run.sh judge ПУТЬ_К_ПРОГОНУ
# например:  ./run.sh judge /home/user/hackathon/data/30618_0e41eac3
```
Windows PowerShell (путь можно скопировать из проводника как есть):
```powershell
.\run.ps1 judge ПУТЬ_К_ПРОГОНУ
# например:  .\run.ps1 judge C:\hackathon\data\30618_0e41eac3
```
Windows Git Bash - путь в стиле Linux: диск `C:\` записывается как `/c/`, обратные слеши заменяются на прямые:
```bash
./run.sh judge /c/hackathon/data/30618_0e41eac3
```
Путь внутрь архива (`...\data.zip\...`) не подходит - сначала распакуйте `data.zip`. Скрипт сам скачает образ
(~1.2 ГБ, один раз), запустит ноду, воспроизведёт bag **в реальном времени**, запишет наши выходные топики и напечатает
метрики относительно GNSS-эталона из того же bag, а также CPU/ОЗУ ноды. Длительность = длительность bag (обычно ~20 мин).

Пример вывода (конец):
```
[judge] инициализация: initialized from GNSS (master): s=10344 m, dist to track 0.5 m
[judge] ресурсы: avg cpu 9.6%, max rss 58.7 MB (266 samples)

==============================================================================
 ИТОГ ПРОВЕРКИ - LCM Solution / tram_odometry
==============================================================================
 Прогон: 30618_0e41eac3   длительность 1327 с   записано сообщений: 51303 velocity, 51303 position

 РЕАЛЬНОЕ ВРЕМЯ И РЕСУРСЫ                 требование        измерено        статус
   частота /result/velocity,/position     >= 10 Гц            38.7 Гц       OK
   задержка вход->публикация, средняя     <= 100 мс           0.45 мс       OK
   задержка вход->публикация, пик         <= 250 мс           1.60 мс       OK
   CPU ноды (среднее)                     <= 200 % (2 ядра)    9.6 %        OK
   память ноды (макс RSS)                 <= 512 МБ           58.7 МБ       OK

 ТОЧНОСТЬ ОТНОСИТЕЛЬНО GNSS-ЭТАЛОНА (эталон: base_link по двум антеннам; сопоставление по метке времени, допуск 0.05 с)
   скорость: RMSE 0.042 м/с   MAE 0.027 м/с   смещение +0.001 м/с   (сопоставлено 51254 сообщений)
   положение (2D): средняя 9.9 м   медиана 12.7 м   RMSE 11.2 м   максимум 28.3 м   (сопоставлено 100 % сообщений)
   конец прогона: ошибка 14.3 м после 5340 м пути  ->  накопленный дрейф 0.27 % дистанции
==============================================================================
```
### Варианты запуска

Команда одна: `run.sh judge ПУТЬ_К_ПРОГОНУ [трамвай] [скорость]`. Можно так:

| Что нужно | Linux / macOS / Git Bash | Windows PowerShell |
|---|---|---|
| Стандартная проверка: трамвай 30618, реальное время (~20 мин) | `./run.sh judge /home/user/data/30618_0e41eac3` | `.\run.ps1 judge C:\data\30618_0e41eac3` |
| Быстрая проверка: воспроизведение в 5 раз быстрее (~4 мин) | `./run.sh judge /home/user/data/30618_0e41eac3 30618 5` | `.\run.ps1 judge C:\data\30618_0e41eac3 30618 5` |
| Трамвай-лаборатория 30639 (своя калибровка колёс и тяги) | `./run.sh judge /home/user/data/30639_d927f360 30639` | `.\run.ps1 judge C:\data\30639_d927f360 30639` |
| Трамвай неизвестен: усреднённая калибровка | `./run.sh judge /home/user/data/30618_0e41eac3 ""` | `.\run.ps1 judge C:\data\30618_0e41eac3 ""` |
| Другой прогон | подставьте другую папку прогона первым аргументом | то же |

- **Трамвай** (2-й аргумент): `30618` - линейный трамвай, по умолчанию; `30639` - трамвай-лаборатория; `""` - усреднённая
  калибровка. Выбор подставляет файл `assets/calib_<id>.yaml`: масштабы одометрии передней и задней тележки и таблицу
  удельного ускорения по позиции контроллера и скорости, идентифицированные по прогонам именно этого трамвая. Карта пути общая.
- **Скорость** (3-й аргумент): `1` - реальное время, как у жюри (по умолчанию); `5` - ускоренное воспроизведение для
  быстрой проверки. Метрики точности при ускорении почти не меняются; замеры задержки и CPU корректны только при `1`.
- Если указать скорость, трамвай тоже нужно указать (аргументы позиционные): `... 30618 5`.

Результаты остаются в `results/`: записанный rosbag2 с `/result/*`, лог ноды (частота и задержка каждые 10 с), лог CPU/ОЗУ.

## Вариант 2: своя проверка (ваш `ros2 bag play` и ваш скрипт-судья)

Запустите только ноду - она слушает `/vehicle/*`, публикует `/result/velocity`, `/result/position`, `/result/diagnostics`:

Linux (нода видна с хоста - сеть хоста, DDS по UDP; `ROS_DOMAIN_ID` берётся из окружения):
```bash
./run.sh node            # = docker run --rm -it --network host ghcr.io/tellsamm/lcm-odometry ros2 launch tram_odometry odometry.launch.py
```
затем на хосте, как обычно: `ros2 bag play <bag>`, `ros2 topic hz /result/position`, ваш скрипт-судья.

Windows / macOS (Docker Desktop без сети хоста) - воспроизведение внутри того же контейнера:
```powershell
.\run.ps1 node                                                         # терминал 1 (контейнер называется lcm)
docker exec -it lcm bash -c "ros2 bag play /data/30618_0e41eac3"       # терминал 2 (нужен -v <папка с bag>:/data:ro при запуске node - см. ниже)
docker exec -it lcm bash -c "ros2 topic hz /result/position"           # терминал 3
```
(для этого запускайте ноду напрямую: `docker run --rm -it --name lcm -v C:\path\to\bags:/data:ro ghcr.io/tellsamm/lcm-odometry ros2 launch tram_odometry odometry.launch.py`).

Параметры: `tram_id:=30639`, `default_start:=stop_T` и остальные - через launch-аргументы или `-p` (см. раздел «Параметры»).

## Вариант 3: внутри проверочного контейнера организаторов (check-code)

Если проверка идёт в контейнере из пакета `check-code` (образ `check-code:humble`, судья `hackathon_solution_checker`),
наш пакет собирается там как есть. Смонтируйте репозиторий в контейнер и запустите готовый скрипт:

```bash
docker run --rm -v /path/to/LCM-Solution:/solution:ro -v /path/to/check-code:/workspace:ro check-code:humble \
  bash /solution/tools/run_in_check_code.sh /workspace/bags/30618_88aea4d9 30618
```
Скрипт копирует `tram_odometry`, наш `tram_vehicle_msgs` и `checker_ros` в рабочее пространство, делает `colcon build`,
запускает нашу ноду и судью, воспроизводит bag и печатает итоговый отчёт судьи. Или те же шаги вручную:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws/src && cp -r /solution/ros2_ws/src/* /workspace/src/checker_ros /tmp/ws/src/
cd /tmp/ws && colcon build --symlink-install && source install/setup.bash
ros2 launch tram_odometry odometry.launch.py &            # наша нода (tram_id:=30618 по умолчанию)
ros2 run hackathon_solution_checker metrics &             # судья
ros2 bag play /workspace/bags/30618_88aea4d9              # воспроизведение
```

Важно про пакет сообщений: в `check-code/src/tram_vehicle_msgs` нет `DriverControllerCommand.msg`, а наша нода
подписана на `/vehicle/driver_position_cmd`. Используйте `tram_vehicle_msgs` из нашего репозитория (он совпадает с
пакетом из основного датасета, добавлено лишь обязательное поле `maintainer`). В одном рабочем пространстве должен
остаться только один пакет с этим именем, иначе `colcon` откажется собирать.

Зависимости ноды - только `rclpy`, `PyYAML` и стандартные интерфейсы (`nav_msgs`, `sensor_msgs`, `diagnostic_msgs`);
всё это есть в `ros:humble-ros-base`. Ни numpy, ни pip-пакетов не требуется.

Наш результат в этом сценарии на bag `30618_88aea4d9` из check-code: скорость RMSE 0.052 м/с, положение 3D RMSE 8.4-9.1 м,
z RMSE 0.14 м (см. `docs/ACCURACY_AND_PERFORMANCE.md`, раздел 2а).

## Вариант 4: без Docker (Ubuntu 22.04 + ROS 2 Humble)

```bash
git clone https://github.com/TellSamm/LCM-Solution.git && cd LCM-Solution/ros2_ws
sudo apt install -y python3-yaml ros-humble-diagnostic-msgs   # обычно уже есть в составе ros-base
colcon build --symlink-install && source install/setup.bash
ros2 launch tram_odometry odometry.launch.py            # tram_id:=30618 по умолчанию
```
Зависимости только стандартные (rclpy, PyYAML, std/nav/sensor/geometry/diagnostic_msgs). Интернет для сборки
пакетов не нужен. Локальная сборка образа, если реестр недоступен: `docker build -t lcm-odometry -f docker/Dockerfile .`
(скрипты `run.sh`/`run.ps1` делают это сами, если `docker pull` не удался).

---

## Что ожидать на выходе

| Топик | Тип | Содержимое |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `velocity` - продольная скорость, **м/с**; `header.stamp` = метка входного сообщения (время bag) |
| `/result/position` | `nav_msgs/msg/Odometry` | `pose.pose.position.{x,y,z}` - метры в локальной плоской СК эталона (UTM 37N минус (300000, 6100000), как в pathgraph); `orientation` - курс; `pose.covariance[0,7]` - дисперсия положения; `twist.twist.linear.x` - скорость; `frame_id=map`, `child_frame_id=base_link` |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `slip_status`: 0 норма / 1 измерение отбраковано (слип, юз, выброс) / 2 датчик завис / 3 нет входных данных; статусы тележек; σ скорости и положения; источник инициализации |

- Метки времени - из `header.stamp` входных сообщений, не из системных часов.
- Публикация на каждое входное сообщение колёс и контроллера (~40 Гц); при пропаже входов до 1 с - по модели (10 Гц),
  дольше - последнее состояние с флагом `status=3`.
- GNSS-топики читаются **только** для начальной выставки (первые секунды, пока трамвай не тронулся) и затем игнорируются.
  Без GNSS нода через 3 с после начала движения стартует из `default_start`.
- QoS подписок - best-effort (совместимо с любым издателем), публикаций - по умолчанию (reliable).

## Задержка, частота, ресурсы

- Задержка «вход -> публикация» измеряется в ноде (от входа в callback до публикации) и печатается в лог каждые 10 с:
  `rate 39 Hz | latency mean 0.4 ms max 1.3 ms`. Типично 0.4-0.6 мс, пик < 5 мс (требование <= 100 мс).
- Частота: `ros2 topic hz /result/position` - ~40 Гц (требование >= 10 Гц).
- Ресурсы: `results/<bag>_resources.log` или `docker stats` - ≈ 10 % одного ядра, ≈ 60 МБ RSS (требование <= 2 ядра, <= 0.5 ГБ).
  `run.sh judge` уже запускает контейнер с `--cpus=2 --memory=512m`.

## Параметры

Все параметры - в `ros2_ws/src/tram_odometry/config/params.yaml`; переопределение:
```bash
ros2 launch tram_odometry odometry.launch.py tram_id:=30639 default_start:=stop_T
ros2 run tram_odometry odometry_node --ros-args -p tram_id:=30618 -p gate_sigma:=3.0
```
Ключевые: `tram_id` (калибровка: `30618` линейный - по умолчанию, `30639` лаборатория, `""` усреднённая),
`default_start` (стартовая точка без GNSS: `stop_S` Щукинская, `stop_T` Таллинская, метры вдоль кольца),
`gnss_init_seconds` (окно начальной выставки). Полное описание - `docs/ASSUMPTIONS_AND_PARAMETERS.md`.

## Примечания

- В приложенном организаторами `tram_vehicle_msgs/package.xml` отсутствовало обязательное поле `<maintainer>`, из-за чего
  `colcon build` завершался ошибкой. В нашем репозитории поле добавлено; определения сообщений не менялись.
- Образ собран из этого репозитория GitHub Actions (`.github/workflows/docker.yml`), Dockerfile - `docker/Dockerfile`
  (база `ros:humble-ros-base`, стандартный `colcon build`).
- Файлы с кириллицей/пробелами в пути к bag могут не монтироваться Docker Desktop - используйте простой путь.
