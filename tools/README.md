# Офлайн-конвейер

Всё, что делается заранее по обучающим bag-файлам: калибровка, карта, оценка точности. Для запуска ноды это не
нужно, готовые результаты уже лежат в `ros2_ws/src/tram_odometry/assets`.

Окружение: Python 3.11, `pip install rosbags numpy scipy pandas pyarrow pyproj pyyaml matplotlib`.
Bag-файлы распакованы в `<repo>/data/<bag_id>/` (или задайте `TRAM_DATA_DIR`). Кэш пишется в `<repo>/cache`.

Шаги по порядку:

```bash
python tools/build_cache.py      # 1. читает все bag один раз, складывает топики в parquet (около 10 с)
python tools/bag_summary.py      # 2. сводка по прогонам: длительность, GNSS, дубликаты -> tools/data/bags_summary.csv
python tools/build_dyn.py        # 3. слитая таблица 10 Гц: скорость и ускорение по GNSS, позиция контроллера, колёса
python tools/calibrate.py        # 4. масштабы колёс и таблицы ускорения a(n, v) -> assets/calib_*.yaml
python tools/build_map.py        # 5. кольцевая карта: pathgraph + петли конечных из GNSS-треков -> assets/track_ring.json
python tools/reparam_ring.py 0.5 # 6. согласование длины дуги в петлях с колёсной одометрией
python tools/fix_loop_z.py       # 7. высота в петлях по медианному профилю GNSS
python tools/evaluate.py --all   # 8. оценка на всех прогонах с GNSS-эталоном (метрики как у жюри)
python tools/tune.py             # 9. перебор параметров фильтра (по желанию)
python tools/make_plots.py       # 10. графики для docs/img
```

`evaluate.py` умеет вносить синтетические аномалии: `--anomaly freeze_both | skid | dropout_long | spikes | noise | slip_front | freeze_front | dropout`.

Внутри контейнера: `run_in_check_code.sh` (запуск в контейнере организаторов check-code), `organizers_check.sh` (прогон через судью организаторов из пакета check-code), `judge_run.sh` (нода + воспроизведение + запись + метрики + ресурсы) и `eval_recorded.py`
(сравнение записанных `/result/*` с GNSS-эталоном из того же bag).
